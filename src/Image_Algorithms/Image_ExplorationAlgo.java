package Image_Algorithms;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.time.LocalTime;
import java.util.ArrayList;
import java.util.stream.IntStream;

import javax.swing.JOptionPane;
import javax.swing.SwingWorker;

import Main.SimulatorMDP;
import Map.Map;
import Map.MapConstants;
import Map.Tile;
import Robot.Robot;
import Robot.Sensor;
import Robot.RobotConstants;
import Robot.RobotConstants.DIRECTION;
import Utils.ConnectionManager;
import Utils.MapDescriptor;

public class Image_ExplorationAlgo {
	
	/*
	 * 1 second = 1000ms.
	 * 1 minute = 60*1000ms.
	 * 6 minutes = 6 * 60 * 1000ms.
	 */
	private boolean specialCase = false; //IMPORTANT - for cases like algotest.txt, set true
	private boolean goCenter = false;

	private boolean diagonalFastest = true;
	
	private int caliThreshold = 5;
	
	private int startCounter = 0;
	private boolean exception = false;
	private boolean cheatRun = false;
	private boolean continueRun=true;
	private long startTime;
	private  long endTime;
	private long timeLimit;
	private  int coverageLimit;
	private int numSteps;
	private int wayX, wayY;
	private static final double one_second = 1000;
	private final int maxCount = 10;
	private String[] imageDetails = null;
	private Map real_map;
	private Map explored_map;
	private FastestPathAlgo fpa;
	private int cur_row, cur_col;
	private int cur_front;
	private ArrayList<String> imageList = new ArrayList<String>();
	private ArrayList<String> imageIdsList = new ArrayList<String>();
	private String imgToSendAnd = "";
	private Robot robot;
	private  int exploredArea=0;
	private ArrayList<Tile> unexploredPath = new ArrayList<Tile>();
	private int LoopCount = 1;
	private int debug_photoCmdCount=0;
	private int moveCount =0;
	//change to user selected
	private long ROBOT_SPEED=10;
	private int strcc = 0;
	public displayManagement displayMgt = null;
	private boolean processImage;
	private Tile waypoint = null;
	private int picture_count = 0;
	private String[] Coord = new String[1000];

	public Image_ExplorationAlgo(Map real_map, Map explored_map, Robot robot, long timeLimit, int coverageLimit, int numSteps,boolean processImage) {
		this.real_map = real_map;
		this.explored_map = explored_map;
		this.robot = robot;
		this.timeLimit = (long) (timeLimit * one_second);
		this.numSteps = numSteps;
		this.coverageLimit = coverageLimit;
		//if processImage is false, then skip all checks for image.
		this.processImage=processImage;
		//this.fpa = new FastestPathAlgo(explored_map,robot,null);

		ROBOT_SPEED = (long) (one_second/numSteps);
		
		//ensure no negative or over coverage limit
		if(coverageLimit<=0 || coverageLimit>300)
			this.coverageLimit = 300;
		//ensure no negative or over 6 mins requirement
		if(timeLimit<=0 || timeLimit >360)
			this.timeLimit = (long) (360*one_second);
	}
	
	/*
	 * To update main display on the simulator
	 */
    public class displayManagement extends SwingWorker<Integer,Void>{
		protected Integer doInBackground() throws Exception{
		    startTime = System.currentTimeMillis();
		    endTime = startTime + timeLimit; 
			while(true) {
				//stop thread condition
				if(processImage) {
					if(System.currentTimeMillis()>=endTime || displayMgt.isCancelled()|| exploredArea>=coverageLimit) {//|| exploredArea>=coverageLimit) {
						displayMgt.cancel(true);
						//Main.SimulatorMDP.exploreWorker.cancel(true);
						break;
					}
				}
				else {
					if(System.currentTimeMillis()>=endTime || displayMgt.isCancelled() || exploredArea>=coverageLimit) {//|| exploredArea>=coverageLimit) {
						displayMgt.cancel(true);
						//Main.SimulatorMDP.exploreWorker.cancel(true);
						break;
					}
				}
				exploredArea = calculateAreaExplored();
				//System.out.println("DEBUG :: Time ->>> "+((endTime - System.currentTimeMillis())/one_second));
				Main.SimulatorMDP.lblTimePassed.setText("Time Left :"+((endTime - System.currentTimeMillis())/one_second)+"s");//;("Time left :"+(endTime - System.currentTimeMillis()) * one_second);
				Main.SimulatorMDP.lblAreaExplored.setText("Area Explored:"+ exploredArea + "/"+ 300);
				Main.SimulatorMDP.lblRobotPos.setText(robot.printLoc());
			}
			continueRun = false;
			return 111;
		}
    }
    
	public void explore() throws IOException {
		this.continueRun=true;
	    System.out.println("Starting exploration...");
	    startExploration();
	}
	/*
	 * 1. Get sensor information
	 * 2. Update information on map
	 * 2.1 Update MDF  
	 * 3. Do algorithm
	 * 3.1 If processImage is true, proceed to do image checking
	 * 4. Send movement to Arduino + coordinates and MDF string to Android
	 * 5. Repeat until reach start zone.
	 * 6. If start zone reach and exploration not fully done, proceed search algorith,.
	 */
	public void startExploration() throws IOException {
	    //this.updateMain();
	    //starts seperate thread for time display management
	    displayMgt = new displayManagement(); //DARREL HERE
	    displayMgt.execute();
	    /*
		 * Exit conditions
		 * 1. Coverage Limit reached
		 * 2. Time Limit reached
		 * 3. Reach back at start (basic right wall will always reach starting point (assuming))
		 */
	    
	    //initial sensor processing
	    this.processSensor(false);
	    //explored_map.paintAgain();
	    
	    //DARREL -> need change while statement?
	    do {
			this.rightWallHug(true,null);
			if(specialCase) {
				if(explored_map.getTile(3, 15).getIsObstacle() && explored_map.getTile(3, 16).getIsObstacle()) {
					exception = true;
				}
			}
			if(this.checkIfStart()) {
				startCounter++;
				System.out.println("startCounter: " + startCounter);
			}
		}while(continueRun && (!this.checkIfStart() || exception) && (startCounter < 2) && (endTime - System.currentTimeMillis() >= 30000)); //exploredArea<=coverageLimit && System.currentTimeMillis() <=endTime && !this.checkIfStart());
		//System.out.println("Sending F to ARD");
		//set up virtual wall before fpa
		explored_map.setObstacleVirtualWall();
		
		System.out.println("Current explored area :"+exploredArea);
		if(!this.checkIfStart() || (endTime - System.currentTimeMillis() < 75000)) {
			//remember to turn it back on
			this.goBackToStartFollowRight();
		}
		else {
			/*
			 * To find if got unexplored area
			 */
			int maxCount = this.maxCount;
			if(this.fpa==null)
				fpa = new FastestPathAlgo(explored_map,robot,null);
			
			//do FPA to path and back to start (bursting) once exploredArea = 300
			/*
			 * Search algorithm to achieve full exploration
			 * General Idea: 
			 * -> Find the nearest unexplored tile
			 * -> Find the closest explorable/ valid tile from it
			 * -> Do A* search algorithm (pathing) to it
			 * -> Move Arduino
			 * -> Repeat until no more valid unexplored tile left
			 */
			while(this.exploredArea!=300) {
				if(SimulatorMDP.exploreWorker.isCancelled()) {
					System.out.println("Exploration is cancelled");
					displayMgt.cancel(true);
					return;
				}

				if(!continueRun)
					break;
				//maxCount--;
				Tile unexplored_temp = explored_map.getNextNotExplored(robot.getRow(), robot.getCol());
				if(unexplored_temp!=null) {
					unexploredPath.add(unexplored_temp);
					Tile temp=explored_map.getNearestExplored(unexplored_temp, robot.getRow(), robot.getCol());
					System.out.println("Tile to go: " + temp.getRow() + "," + temp.getCol());
					boolean explor = false;
					boolean cnt = false;
					ArrayList<Tile> tempPath = this.fpa.findFastestPath(robot.getRow(), robot.getCol(), temp.getRow(), temp.getCol(), robot.getCurrentHeading(), explor, cnt);
					
					if (tempPath == null || tempPath.size()==0 ||
							maxCount==0) {
						//this.goBackToStartFollowRight();
						break;
					} else {
						Tile start = explored_map.getTile(robot.getRow(), robot.getCol());
						ArrayList<RobotConstants.DIRECTION> list_direction =this.moveFPA(tempPath,start,robot.getCurrentHeading());	
						if(list_direction!=null)
							this.moveWithList(list_direction,false);
						
						fpa.printGMap();
						
						if(explored_map.getTile(unexplored_temp.getRow(), unexplored_temp.getCol()).getIsExplored())
							unexploredPath.remove(explored_map.getTile(unexplored_temp.getRow(), unexplored_temp.getCol()));
						else
							explored_map.getTile(unexplored_temp.getRow(), unexplored_temp.getCol()).setIsDiscoverble(false);
					}
					//fpa.setMap(explored_map);
					explored_map.setObstacleVirtualWall();
					explored_map.paintAgain();
					//this.updateMain();
				
				}
				else {
					int remainingCoverage = MapConstants.MAP_ROWS * MapConstants.MAP_COLS -this.exploredArea;
					if(remainingCoverage-unexploredPath.size()==0)
						break;
				}
				if(unexplored_temp==null)
				break;
			}
			System.out.println("Finding path back to start full explore");
			if(!this.checkIfStart()){
				this.goBackToStartFollowRight();
			}
			else {
				while(this.robot.getCurrentHeading() != RobotConstants.HEADING.NORTH)
				this.turnRobot('R',true);
			}			
		}
		displayMgt.cancel(true);
		System.out.println("Command Count : "+this.debug_photoCmdCount);
		System.out.println("Possible points :"+this.explored_map.getPossiblePoints().size());
    	for(int c =MapConstants.MAP_COLS-1; c>=0;c--) {
    		for(int r =0 ; r< MapConstants.MAP_ROWS;r++) {
    			Tile temp = explored_map.getTile(r,c);
    			if(temp.getIsObstacle())
    				temp.checkAll4();
    		}
    	}


    	/*
    	 * 'Island' Hugging algorithm. FOR IR. Did not use
    	 * Basic Idea: Find clusters of obstacles in the center of the arena
    	 * Perform Right-Wall hug on all clusters
    	 */
    	if(processImage && goCenter) {
			continueRun = true;
			if (continueRun) {
				System.out.println("OUT");
				explored_map.paintAgain();
				explored_map.setToBeExplored();
				ArrayList<ArrayList<Tile>> tempList = explored_map.getToBeExplored();
				for (int i = 0; i < tempList.size(); i++) {
					System.out.println("ISLAND :" + i);
					System.out.println("SIZE OF NEXT LOOP :" + tempList.get(i).size());
					for (int y = 0; y < tempList.get(i).size(); y++) {
						System.out.println(tempList.get(i).get(y).printLoc());
					}
					System.out.println("==========================================================");
				}
				if (this.fpa == null)
					fpa = new FastestPathAlgo(explored_map, robot, null);

				for (int i = 0; i < tempList.size(); i++) {
//    		if(tempList.get(i).size()<=1)
//    			continue;
					if (this.fpa == null)
						fpa = new FastestPathAlgo(explored_map, robot, null);

					Tile temp = explored_map.getNearestIslandPoint(tempList.get(i).get(0), robot.getRow(), robot.getCol());
					System.out.println("Tile to go: " + temp.getRow() + "," + temp.getCol());
					ArrayList<Tile> tempPath = this.fpa.findFastestPath(robot.getRow(), robot.getCol(), temp.getRow(), temp.getCol(), robot.getCurrentHeading(), false, false);

					if (tempPath == null || tempPath.size() == 0 ||
							maxCount == 0) {
						//this.goBackToStartFollowRight();
						break;
					} else {
						Tile start = explored_map.getTile(robot.getRow(), robot.getCol());
						ArrayList<RobotConstants.DIRECTION> list_direction = this.moveFPA(tempPath, start, robot.getCurrentHeading());
						if (list_direction != null) {
							this.moveWithList(list_direction, false);

							//reach the point
							this.makeMeTurnCorrect(tempList.get(i).get(0));
//					if(this.checkFront()!=-1) {
//						this.turnRobot('L',false);
//						if(this.checkRObs() == 2) {
//							this.sendTakePhotoCmd();
//						}
//						if(this.checkFront()==-1)
//							this.moveRobot(false);
//					}

							while (continueRun) {
								this.rightWallHug(false, temp);//explored_map.getTile(robot.getRow(), robot.getCol()));
								if (robot.getRow() == temp.getRow() && robot.getCol() == temp.getCol())
									break;


							}
							if (!continueRun)
								break;
							//Break point
							//if(cur_row == ref.getRow() && cur_col==ref.getCol())
							//return;
							//}

							//this.rightWallHug(false,explored_map.getTile(robot.getRow(), robot.getCol()));
						}

					}
				}

				if (!this.checkIfStart())
					this.goBackToStartFollowRight();

				displayMgt.cancel(true);
			}
		}
		/*
		 * added the image preocessing algorithm here!!! Process all the image after the exploration
		 */
		System.out.println("Start processing all the images");
		Process p = Runtime.getRuntime().exec("sh /Users/hannancao/Desktop/CZ3004/Updated_Group_2_Algorithm_Codes/src/Main/image.sh");
		BufferedReader stdInput = new BufferedReader(new
				InputStreamReader(p.getInputStream()));

		BufferedReader stdError = new BufferedReader(new
				InputStreamReader(p.getErrorStream()));

		// read the output from the command
		System.out.println("Here is the standard output of the command:\n");
		String s = null;
		int [] label = IntStream.range(0, 100).toArray();
		int count = 0;
		while ((s = stdInput.readLine()) != null) {
			System.out.println(s);
			if (s.length() < 9){
				String[] arrOfStr = s.split("/", 4);
				if (arrOfStr.length == 3){
					System.out.println("Image ID received:");
					System.out.println(arrOfStr[0]);
					String x_coor = arrOfStr[1];
					String y_coor = arrOfStr[2];

					String msg = x_coor+":"+y_coor;

					processImageReceive(msg, arrOfStr[0]);
					System.out.println("The string sequence to pass to the Andriod");
					System.out.println(imgToSendAnd);
					String mdfString = this.updateMap();

					if(this.imgToSendAnd=="")
						ConnectionManager.getConnMgr().sendMessage(mdfString+":"+robot.getRow()+":"+(Math.abs(robot.getCol()-MapConstants.MAP_COLS)-1)+":"+"N", "AND");
					else
						ConnectionManager.getConnMgr().sendMessage(mdfString+":"+robot.getRow()+":"+(Math.abs(robot.getCol()-MapConstants.MAP_COLS)-1)+":"+"N"+":"+this.imgToSendAnd, "AND");
				}
			}
			else if (s.equalsIgnoreCase("Ignore")){
				count += 1;
			}
		}
		System.out.println("Image Captured!!!!!!!!!!!!:");
		System.out.println(label[count]);
		// read any errors from the attempted command
		System.out.println("Here is the standard error of the command (if any):\n");
		String e = null;
		while ((e = stdError.readLine()) != null) {
			System.out.println(e);
		}

		// pass the labels to the Process:
		for (int i = 0; i < count; i++){
			if (label[i] != 0){
				processImageReceive(Coord[i], Integer.toString(label[i]));
			}
		}
		for (int i = 0; i < count; i++){
			System.out.println(label[i]);
		}


		// combine images together

		System.out.println("Start processing all the images");
		Process comb_p = Runtime.getRuntime().exec("sh /Users/hannancao/Desktop/CZ3004/Updated_Group_2_Algorithm_Codes/src/Main/combine.sh");
		BufferedReader stdInput_tem = new BufferedReader(new
				InputStreamReader(comb_p.getInputStream()));

		BufferedReader stdError_tem = new BufferedReader(new
				InputStreamReader(comb_p.getErrorStream()));
    	
	}
	
	/*
	 * For IR island-hugging.
	 * Turns the robot to face correctly in order to perform right-wall hugging
	 */
	private void makeMeTurnCorrect(Tile ref) {
		// TODO Auto-generated method stub
		RobotConstants.HEADING h = robot.getCurrentHeading();
		switch(h) {
		case NORTH:
			if(robot.getRow()==ref.getRow()) {
				if(robot.getCol()>ref.getCol()) {
					this.turnRobot('L', false);
					if(this.checkFront()==-1)
						this.moveRobot(false, true);
				}
				
			}
			if(robot.getRow()>ref.getRow()) {
				if(robot.getCol()>=ref.getCol()) {
					this.turnRobot('L',false);
					this.turnRobot('L',false);
					
				}
				else {
				this.turnRobot('L', false);
				if(this.checkFront()==-1)
					this.moveRobot(false, true);
				}
			}
			else {
				if(this.checkR()!=-1)
					break;
				if(this.checkR()==-1) {
					while(this.checkR()==-1) {
						if(this.checkFront()==-1)
							moveRobot(false, true);
						else break;
					}
				}
				else {
				this.turnRobot('L', false);
				if(this.checkFront()==-1)
					this.moveRobot(false, true);
				}
			}
			break;
		case SOUTH:
			if(robot.getRow()<ref.getRow()) {
				this.turnRobot('L',false);
				this.turnRobot('L',false);
			}
			break;
		case EAST:
			if(robot.getCol()>=ref.getCol()) {
				this.turnRobot('L',false);
				this.turnRobot('L',false);
			}
			//else if(robot.getCol()<ref.getCol())
			break;
		case WEST:
			if(robot.getRow()<=ref.getRow()) {
				if(robot.getCol()<ref.getCol()) {
				this.turnRobot('L',false);
				this.turnRobot('L',false);
				}
			}
			break;
		}
		
	}

	private void stopRun() {
		// TODO Auto-generated method stub
		System.out.println("Exploration is cancelled");
		displayMgt.cancel(true);
		explored_map.setObstacleVirtualWall();
		explored_map.paintAgain();
	}

	/*
	 * ignoreSensors control the need to burst or just move regularly while updating the map
	 */
	private void moveWithList(ArrayList<DIRECTION> list_direction, boolean ignoreSensor) {
		// TODO Auto-generated method stub
		if(!ignoreSensor) {
		for(RobotConstants.DIRECTION e : list_direction) {
			switch(e) {
			case FRONT:
				this.moveRobot(ignoreSensor, false);//robot.moveRobot(robot.getCurrentHeading(),explored_map);
				break;
			case LEFT:
				this.turnRobot('L',ignoreSensor);
				//robot.turningRobot('L', explored_map);
				break;
			case RIGHT:
				this.turnRobot('R',ignoreSensor);
				//robot.turningRobot('R', explored_map);
				break;
			}
		}
		}
		else {
			for(RobotConstants.DIRECTION e : list_direction) {
				switch(e) {
				case FRONT:
					robot.moveRobot(robot.getCurrentHeading(),explored_map);
					//this.moveRobot(ignoreSensor);//robot.moveRobot(robot.getCurrentHeading(),explored_map);
					break;
				case LEFT:
					//this.turnRobot('L',ignoreSensor);
					robot.turningRobot('L', explored_map);
					break;
				case RIGHT:
					//this.turnRobot('R',ignoreSensor);
					robot.turningRobot('R', explored_map);
					break;
				}
				//this.explored_map.paintAgain();
			}
		}
	}

	/*
	 * Find fastest path back to start using A* search
	 */
	public void goBackToStartFollowRight() {
		System.out.println("Going back following path");
		if(this.fpa==null)
			fpa = new FastestPathAlgo(explored_map,robot,null);

		boolean explor = true;
		boolean cnt = false;
		fpa.resetCost();
		ArrayList<Tile> pathToStart = fpa.findFastestPath(robot.getRow(), robot.getCol(), RobotConstants.START_POS_X, RobotConstants.START_POS_Y, robot.getCurrentHeading(), explor, cnt);
		//this.moveFPA(pathToStart, false,true);
		Tile start = explored_map.getTile(robot.getRow(), robot.getCol());
		ArrayList<RobotConstants.DIRECTION> list_direction =this.moveFPA(pathToStart,start,robot.getCurrentHeading());	
		if(list_direction!=null) {
			String movementString = this.getMovementString(list_direction, false);
			if(robot.getRealRun()) {
				ConnectionManager.getConnMgr().sendMessage(movementString, "ARD");	
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				//ConnectionManager.getConnMgr().sendMessage(movementString, "AND");
			}
			this.moveWithList(list_direction,true);
			String mdfString = this.updateMap();
			//ConnectionManager.getConnMgr().sendMessage(mdfString+":"+robot.getRow()+":"+(Math.abs(robot.getCol()-MapConstants.MAP_COLS)-1)+":"+"N", "AND");
			if(this.imgToSendAnd=="")
				ConnectionManager.getConnMgr().sendMessage(mdfString+":"+robot.getRow()+":"+(Math.abs(robot.getCol()-MapConstants.MAP_COLS)-1)+":"+"N", "AND");
			else
				ConnectionManager.getConnMgr().sendMessage(mdfString+":"+robot.getRow()+":"+(Math.abs(robot.getCol()-MapConstants.MAP_COLS)-1)+":"+"N"+":"+this.imgToSendAnd, "AND");
			
		}
		//delay to prevent message concatenation
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		while(true) {  //DARREL CALIBRATE
			System.out.println("Entered here!");
			boolean calibrated = false;
			if(robot.getCurrentHeading() == RobotConstants.HEADING.SOUTH && this.checkRCalibrate()==-1 && !calibrated) {
				System.out.println("Starting to CALIBRATE RIGHT");
				if(robot.getRealRun()) {
					ConnectionManager.getConnMgr().sendMessage("H", "ARD");
					calibrated = true;
				}
			}
			else if(robot.getCurrentHeading() == RobotConstants.HEADING.WEST && this.checkLCalibrate()==-1 && !calibrated) {
				System.out.println("Starting to CALIBRATE LEFT");
				if(robot.getRealRun()) {
					ConnectionManager.getConnMgr().sendMessage("Q", "ARD");
					calibrated = true;
				}
			}
			this.turnRobot('R',true);
			if(robot.getCurrentHeading() == RobotConstants.HEADING.NORTH)
				break;
		}
//		displayMgt.cancel(true);
//		System.out.println("Cancel");
		fpa.printGMap();
	}
	
	/*
	 * Algorithm for fastest path
	 * General Idea:
	 * -> If waypoint is set, find path to waypoint first.
	 * -> Find path to goal zone from current point.
	 * -> Concatenate both path (if exist).
	 * -> Generate movement string from list of paths.
	 * -> Send to Ard and And.
	 */
	public void startFastestPathAlgo() {
		System.out.println("DEBUG :: Finding Path...");
		fpa.resetCost();
		int end_row = RobotConstants.END_POS_X;
		int end_col = RobotConstants.END_POS_Y;
		int start_row = RobotConstants.START_POS_X;
		int start_col = RobotConstants.START_POS_Y;
		boolean explor = false;
		String movementString="";
		this.robot.setRow(start_row);
		this.robot.setCol(start_col);
		this.robot.setHeading(RobotConstants.HEADING.NORTH);
		explored_map.paintAgain();
    	robot = new Robot(RobotConstants.START_POS_X, RobotConstants.START_POS_Y, robot.getRealRun(), 0);
		explored_map.setRobot(robot);
		
		RobotConstants.HEADING curH = robot.getCurrentHeading();
		waypoint = explored_map.getWaypoint();
		FastestPathAlgo fpa = new FastestPathAlgo(explored_map,robot,waypoint);
		fpa.setPathNotTrue();
		if(this.exploredArea == 300) {
			end_row = RobotConstants.END_POS_X;
			end_col = RobotConstants.END_POS_Y;
		}
		ArrayList<RobotConstants.DIRECTION> list_direction;
		if(waypoint==null) {
			Tile start = explored_map.getTile(start_row, start_col);
			boolean cnt = false;
			ArrayList <Tile> path = fpa.findFastestPath(start_row, start_col, end_row, end_col, curH, explor, cnt);
			list_direction =this.moveFPA(path,start,curH);	

		}else {
			int ori_row = waypoint.getRow();
			int ori_col = waypoint.getCol();
			ArrayList<RobotConstants.DIRECTION> best_direction = null;
			int step_list[] = {999, 999, 999, 999, 999, 999, 999};
			if(explored_map.checkVirtualWall(ori_row, ori_col)){
				boolean cnt = true;
				if (!explored_map.checkVirtualWall(ori_row+1, ori_col-1) && !explored_map.checkWall(ori_row+1, ori_col-1)){
					waypoint.setRow(ori_row+1);
					waypoint.setCol(ori_col-1);
					ArrayList<Tile> pathtowp = fpa.findFastestPath(start_row, start_col, waypoint.getRow(), waypoint.getCol(), curH, explor, true);
					fpa.printGMap();
					//moveFPA(pathtowp,true,false);
					Tile start = explored_map.getTile(start_row, start_col);
					Tile temp = pathtowp.get(pathtowp.size() - 2);
					RobotConstants.HEADING tempH = fpa.getNextHeading(temp, waypoint.getRow(), waypoint.getCol(), curH);
					ArrayList<Tile> pathtogoal = fpa.findFastestPath(waypoint.getRow(), waypoint.getCol(), end_row, end_col, tempH, explor, true);//robot.getCurrentHeading());
					//moveFPA(pathtogoal,true,false);
					int len = pathtowp.size() + pathtogoal.size();
					step_list[0] = len;
					waypoint.setRow(ori_row);
					waypoint.setCol(ori_col);
				}
				if(!explored_map.checkVirtualWall(ori_row, ori_col-1) && !explored_map.checkWall(ori_row, ori_col-1)){
					waypoint.setRow(ori_row);
					waypoint.setCol(ori_col-1);
					ArrayList<Tile> pathtowp = fpa.findFastestPath(start_row, start_col, waypoint.getRow(), waypoint.getCol(), curH, explor, true);
					fpa.printGMap();
					//moveFPA(pathtowp,true,false);
					Tile start = explored_map.getTile(start_row, start_col);
					Tile temp = pathtowp.get(pathtowp.size() - 2);
					RobotConstants.HEADING tempH = fpa.getNextHeading(temp, waypoint.getRow(), waypoint.getCol(), curH);
					ArrayList<Tile> pathtogoal = fpa.findFastestPath(waypoint.getRow(), waypoint.getCol(), end_row, end_col, tempH, explor, true);//robot.getCurrentHeading());
					//moveFPA(pathtogoal,true,false);
					int len = pathtowp.size() + pathtogoal.size();
					step_list[1] = len;
					waypoint.setRow(ori_row);
					waypoint.setCol(ori_col);
				}
				if(!explored_map.checkVirtualWall(ori_row-1, ori_col-1) && !explored_map.checkWall(ori_row-1, ori_col-1)){
					waypoint.setRow(ori_row-1);
					waypoint.setCol(ori_col-1);
					ArrayList<Tile> pathtowp = fpa.findFastestPath(start_row, start_col, waypoint.getRow(), waypoint.getCol(), curH, explor, true);
					fpa.printGMap();
					//moveFPA(pathtowp,true,false);
					Tile start = explored_map.getTile(start_row, start_col);
					Tile temp = pathtowp.get(pathtowp.size() - 2);
					RobotConstants.HEADING tempH = fpa.getNextHeading(temp, waypoint.getRow(), waypoint.getCol(), curH);
					ArrayList<Tile> pathtogoal = fpa.findFastestPath(waypoint.getRow(), waypoint.getCol(), end_row, end_col, tempH, explor, true);//robot.getCurrentHeading());
					//moveFPA(pathtogoal,true,false);
					int len = pathtowp.size() + pathtogoal.size();
					step_list[2] = len;
					waypoint.setRow(ori_row);
					waypoint.setCol(ori_col);
				}
				if(!explored_map.checkVirtualWall(ori_row-1, ori_col) && !explored_map.checkWall(ori_row-1, ori_col)){
					waypoint.setRow(ori_row-1);
					waypoint.setCol(ori_col);
					ArrayList<Tile> pathtowp = fpa.findFastestPath(start_row, start_col, waypoint.getRow(), waypoint.getCol(), curH, explor, true);
					fpa.printGMap();
					//moveFPA(pathtowp,true,false);
					Tile start = explored_map.getTile(start_row, start_col);
					Tile temp = pathtowp.get(pathtowp.size() - 2);
					RobotConstants.HEADING tempH = fpa.getNextHeading(temp, waypoint.getRow(), waypoint.getCol(), curH);
					ArrayList<Tile> pathtogoal = fpa.findFastestPath(waypoint.getRow(), waypoint.getCol(), end_row, end_col, tempH, explor, true);//robot.getCurrentHeading());
					//moveFPA(pathtogoal,true,false);
					int len = pathtowp.size() + pathtogoal.size();
					step_list[3] = len;
					waypoint.setRow(ori_row);
					waypoint.setCol(ori_col);
				}
				if(!explored_map.checkVirtualWall(ori_row-1, ori_col+1) && !explored_map.checkWall(ori_row-1, ori_col+1)){
					waypoint.setRow(ori_row-1);
					waypoint.setCol(ori_col+1);
					ArrayList<Tile> pathtowp = fpa.findFastestPath(start_row, start_col, waypoint.getRow(), waypoint.getCol(), curH, explor, true);
					fpa.printGMap();
					//moveFPA(pathtowp,true,false);
					Tile start = explored_map.getTile(start_row, start_col);
					Tile temp = pathtowp.get(pathtowp.size() - 2);
					RobotConstants.HEADING tempH = fpa.getNextHeading(temp, waypoint.getRow(), waypoint.getCol(), curH);
					ArrayList<Tile> pathtogoal = fpa.findFastestPath(waypoint.getRow(), waypoint.getCol(), end_row, end_col, tempH, explor, true);//robot.getCurrentHeading());
					//moveFPA(pathtogoal,true,false);
					int len = pathtowp.size() + pathtogoal.size();
					step_list[4] = len;
					waypoint.setRow(ori_row);
					waypoint.setCol(ori_col);
				}
				if(!explored_map.checkVirtualWall(ori_row, ori_col+1) && !explored_map.checkWall(ori_row, ori_col+1)){
					waypoint.setRow(ori_row);
					waypoint.setCol(ori_col+1);
					ArrayList<Tile> pathtowp = fpa.findFastestPath(start_row, start_col, waypoint.getRow(), waypoint.getCol(), curH, explor, true);
					fpa.printGMap();
					//moveFPA(pathtowp,true,false);
					Tile start = explored_map.getTile(start_row, start_col);
					Tile temp = pathtowp.get(pathtowp.size() - 2);
					RobotConstants.HEADING tempH = fpa.getNextHeading(temp, waypoint.getRow(), waypoint.getCol(), curH);
					ArrayList<Tile> pathtogoal = fpa.findFastestPath(waypoint.getRow(), waypoint.getCol(), end_row, end_col, tempH, explor, true);//robot.getCurrentHeading());
					//moveFPA(pathtogoal,true,false);
					int len = pathtowp.size() + pathtogoal.size();
					step_list[5] = len;
					waypoint.setRow(ori_row);
					waypoint.setCol(ori_col);
				}
				if(!explored_map.checkVirtualWall(ori_row+1, ori_col+1) && !explored_map.checkWall(ori_row+1, ori_col+1)){
					waypoint.setRow(ori_row+1);
					waypoint.setCol(ori_col+1);
					ArrayList<Tile> pathtowp = fpa.findFastestPath(start_row, start_col, waypoint.getRow(), waypoint.getCol(), curH, explor, true);
					fpa.printGMap();
					//moveFPA(pathtowp,true,false);
					Tile start = explored_map.getTile(start_row, start_col);
					Tile temp = pathtowp.get(pathtowp.size() - 2);
					RobotConstants.HEADING tempH = fpa.getNextHeading(temp, waypoint.getRow(), waypoint.getCol(), curH);
					ArrayList<Tile> pathtogoal = fpa.findFastestPath(waypoint.getRow(), waypoint.getCol(), end_row, end_col, tempH, explor, true);//robot.getCurrentHeading());
					//moveFPA(pathtogoal,true,false);

					int len = pathtowp.size() + pathtogoal.size();
					step_list[6] = len;
					waypoint.setRow(ori_row);
					waypoint.setCol(ori_col);
				}
				if(!explored_map.checkVirtualWall(ori_row+1, ori_col) && !explored_map.checkWall(ori_row+1, ori_col)){
					waypoint.setRow(ori_row+1);
					waypoint.setCol(ori_col);
					ArrayList<Tile> pathtowp = fpa.findFastestPath(start_row, start_col, waypoint.getRow(), waypoint.getCol(), curH, explor, true);
					fpa.printGMap();
					//moveFPA(pathtowp,true,false);
					Tile start = explored_map.getTile(start_row, start_col);
					Tile temp = pathtowp.get(pathtowp.size() - 2);
					RobotConstants.HEADING tempH = fpa.getNextHeading(temp, waypoint.getRow(), waypoint.getCol(), curH);
					ArrayList<Tile> pathtogoal = fpa.findFastestPath(waypoint.getRow(), waypoint.getCol(), end_row, end_col, tempH, explor, true);//robot.getCurrentHeading());
					//moveFPA(pathtogoal,true,false);
					int len = pathtowp.size() + pathtogoal.size();
					step_list[7] = len;
					waypoint.setRow(ori_row);
					waypoint.setCol(ori_col);
				}
				int ind = 0;
				for(int i = 0; i < 7; i++){
					if(step_list[i] < step_list[ind])
						ind = i;
				}
				if(ind==0){
					waypoint.setRow(ori_row+1);
					waypoint.setCol(ori_col-1);
				}
				else if(ind==1){
					waypoint.setRow(ori_row);
					waypoint.setCol(ori_col-1);
				}
				else if(ind==2){
					waypoint.setRow(ori_row-1);
					waypoint.setCol(ori_col-1);
				}
				else if(ind==3){
					waypoint.setRow(ori_row-1);
					waypoint.setCol(ori_col);
				}
				else if(ind==4){
					waypoint.setRow(ori_row-1);
					waypoint.setCol(ori_col+1);
				}
				else if(ind==5){
					waypoint.setRow(ori_row);
					waypoint.setCol(ori_col+1);
				}
				else if(ind==6){
					waypoint.setRow(ori_row+1);
					waypoint.setCol(ori_col+1);
				}
				else if(ind==7){
					waypoint.setRow(ori_row+1);
					waypoint.setCol(ori_col);
				}

				boolean not_realrun = false;
				System.out.println(ind);
				System.out.println(step_list);
				ArrayList<Tile> pathtowp = fpa.findFastestPath(start_row, start_col, waypoint.getRow(), waypoint.getCol(), curH, explor, not_realrun);
				fpa.printGMap();
				//moveFPA(pathtowp,true,false);
				Tile start = explored_map.getTile(start_row, start_col);
				Tile temp = pathtowp.get(pathtowp.size() - 2);
				RobotConstants.HEADING tempH = fpa.getNextHeading(temp, waypoint.getRow(), waypoint.getCol(), curH);
				list_direction = this.moveFPA(pathtowp, start, curH);
				ArrayList<Tile> pathtogoal = fpa.findFastestPath(waypoint.getRow(), waypoint.getCol(), end_row, end_col, tempH, explor, not_realrun);//robot.getCurrentHeading());
				//moveFPA(pathtogoal,true,false);
				ArrayList<RobotConstants.DIRECTION> list_direction2 = this.moveFPA(pathtogoal, waypoint, tempH);
				list_direction.addAll(list_direction2);
//				ArrayList<Tile> pathtowp = fpa.findFastestPath(start_row, start_col, waypoint.getRow(), waypoint.getCol(), curH, explor);
//				fpa.printGMap();
//				//moveFPA(pathtowp,true,false);
//				Tile start = explored_map.getTile(start_row, start_col);
//				Tile temp = pathtowp.get(pathtowp.size() - 2);
//				RobotConstants.HEADING tempH = fpa.getNextHeading(temp, waypoint.getRow(), waypoint.getCol(), curH);
//				list_direction = this.moveFPA(pathtowp, start, curH);
//				ArrayList<Tile> pathtogoal = fpa.findFastestPath(waypoint.getRow(), waypoint.getCol(), end_row, end_col, tempH, explor);//robot.getCurrentHeading());
//				//moveFPA(pathtogoal,true,false);
//				ArrayList<RobotConstants.DIRECTION> list_direction2 = this.moveFPA(pathtogoal, waypoint, tempH);
//				list_direction.addAll(list_direction2);
			}
			else{
				boolean cnt_again = false;
				System.out.println("Generating Here");
				ArrayList<Tile> pathtowp = fpa.findFastestPath(start_row, start_col, waypoint.getRow(), waypoint.getCol(), curH, explor, cnt_again);
				fpa.printGMap();
				//moveFPA(pathtowp,true,false);
				Tile start = explored_map.getTile(start_row, start_col);
				Tile temp = pathtowp.get(pathtowp.size() - 2);
				RobotConstants.HEADING tempH = fpa.getNextHeading(temp, waypoint.getRow(), waypoint.getCol(), curH);
				list_direction = this.moveFPA(pathtowp, start, curH);
				ArrayList<Tile> pathtogoal = fpa.findFastestPath(waypoint.getRow(), waypoint.getCol(), end_row, end_col, tempH, explor, cnt_again);//robot.getCurrentHeading());
				//moveFPA(pathtogoal,true,false);
				ArrayList<RobotConstants.DIRECTION> list_direction2 = this.moveFPA(pathtogoal, waypoint, tempH);
				
				list_direction.add(RobotConstants.DIRECTION.STILL);
				
				list_direction.addAll(list_direction2);
			}
//				ArrayList<Tile> pathtowp = fpa.findFastestPath(start_row, start_col, waypoint.getRow(), waypoint.getCol(), curH, explor);
//				fpa.printGMap();
//				//moveFPA(pathtowp,true,false);
//				Tile start = explored_map.getTile(start_row, start_col);
//				Tile temp = pathtowp.get(pathtowp.size() - 2);
//				RobotConstants.HEADING tempH = fpa.getNextHeading(temp, waypoint.getRow(), waypoint.getCol(), curH);
//				list_direction = this.moveFPA(pathtowp, start, curH);
//				ArrayList<Tile> pathtogoal = fpa.findFastestPath(waypoint.getRow(), waypoint.getCol(), end_row, end_col, tempH, explor);//robot.getCurrentHeading());
//				//moveFPA(pathtogoal,true,false);
//				ArrayList<RobotConstants.DIRECTION> list_direction2 = this.moveFPA(pathtogoal, waypoint, tempH);
//				list_direction.addAll(list_direction2);

		}
		fpa.printGMap();
		
		if(robot.getRealRun()) {
			while(true) {
				System.out.println("WAITING FOR FP FROM AND");
				String msg = ConnectionManager.getConnMgr().receiveMessage();
				if(msg!=null) {
					if(msg.equalsIgnoreCase("FP"))
						break;
				}
			}
		}
		else {
			System.out.println("FP Received");
		}
		

		if(list_direction!=null) {
			movementString = this.getMovementString(list_direction, true);
			if(robot.getRealRun()) {
				ConnectionManager.getConnMgr().sendMessage(movementString, "ARD");
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				ConnectionManager.getConnMgr().sendMessage(movementString, "AND");
			}
		}
		if(!robot.getRealRun()){
			this.moveWithList(list_direction,false);
		}
	}
	
	/*
	 * To generate movement string for fastest path/ bursting
	 * F, 0-9 : Number of forward movements 
	 * R : Right Turn
	 * L : Left Turn
	 */
	private String getMovementString(ArrayList<DIRECTION> list_direction, boolean startToGoal) {
		// TODO Auto-generated method stub
		StringBuilder sb = new StringBuilder();
		int front_movement =-1;
		for(RobotConstants.DIRECTION e : list_direction) {
			switch(e) {
			case FRONT:
				front_movement++;
				//sb.append("F");
				break;
			case LEFT:
				if(front_movement==0)
					sb.append(0);
				else if(front_movement>9) {
					sb.append(9);
					sb.append((front_movement-10)==0 ? 0 : (front_movement-10));
				}
				else if(front_movement!=-1)
					sb.append(front_movement);

				sb.append("L");
				front_movement=-1;
				break;
			case RIGHT:
				if(front_movement==0)
					sb.append(0);
				else if(front_movement>9) {
					sb.append(9);
					sb.append((front_movement-10)==0 ? 0 : (front_movement-10));
				}
				else if(front_movement!=-1)
					sb.append(front_movement);

				sb.append("U");
				front_movement=-1;
				break;
			case STILL:
				if(front_movement==0)
					sb.append(0);
				else if(front_movement>9) {
					sb.append(9);
					sb.append((front_movement-10)==0 ? 0 : (front_movement-10));
				}
				else if(front_movement!=-1)
					sb.append(front_movement);

				sb.append("!");
				front_movement=-1;
				break;
			}
		}
		if(front_movement==0)
			sb.append(0);
		else if(front_movement>9) {
			sb.append(9);
			sb.append((front_movement-10)==0 ? 0 : (front_movement-10));
		}
		else if(front_movement!=-1)
			sb.append(front_movement);
		System.out.println("DEBUG :: Movement String -> "+sb.toString());
		
		if(!startToGoal) {
			int offset=0;
			for(int i=0; i<sb.length(); i++) {
				if(i+offset+1<=sb.length()) {
					if(sb.substring(i+offset,i+offset+1).compareToIgnoreCase("L") == 0) {
						sb.replace(i+offset, i+offset+1, "HL");
						offset++;
						i+=1;
					}
					if(sb.substring(i+offset,i+offset+1).compareToIgnoreCase("U") == 0) {
						sb.replace(i+offset, i+offset+1, "HU");
						offset++;
						i+=1;
					}
				}
			}
		}
		
		if(startToGoal && !robot.getRealRun()) {
			String diagonalStr = this.diagonalAfterWaypoint(sb.toString());
			System.out.println("Diagonal String -> " + diagonalStr);
			int index = sb.indexOf("!");
			int skip = this.numCharToSkipIfFront(diagonalStr);
			if(skip == 0)
				System.out.println("Total Movement String -> " + sb.substring(0, index) + diagonalStr);
			else {
				if(!this.isNumeric(sb.substring(index-1, index))){
					System.out.println("Total Movement String -> " + sb.substring(0, index) + diagonalStr);
				}
				else {
					StringBuilder sbMid = new StringBuilder();
					int dist = this.goFrontDist(diagonalStr.substring(0, skip)) + Integer.valueOf(sb.substring(index-1, index))+1;
					if(dist==0) {
					}
					else if(dist>20) {
						sbMid.append(9);
						sbMid.append(9);
						sbMid.append((dist-21) == 0 ? 0 : (dist-21));
					}
					else if(dist>10) {
						sbMid.append(9);
						sbMid.append((dist-11) == 0 ? 0 : (dist-11));
					}
					else {
						sbMid.append(dist-1);
					}
					System.out.println("Total Movement String -> " + sb.substring(0,index-1) + sbMid.toString() + diagonalStr.substring(skip));
				}
			}
		}
		
		if(startToGoal && robot.getRealRun() && diagonalFastest) {
			String diagonalStr = this.diagonalAfterWaypoint(sb.toString());
			System.out.println("Diagonal String -> " + diagonalStr);
			int index = sb.indexOf("!");
			int skip = this.numCharToSkipIfFront(diagonalStr);
			if(skip == 0)
				return sb.substring(0, index) + diagonalStr;
			else {
				if(!this.isNumeric(sb.substring(index-1, index))){
					return sb.substring(0, index) + diagonalStr;
				}
				else {
					StringBuilder sbMid = new StringBuilder();
					int dist = this.goFrontDist(diagonalStr.substring(0, skip)) + Integer.valueOf(sb.substring(index-1, index))+1;
					if(dist==0) {
					}
					else if(dist>20) {
						sbMid.append(9);
						sbMid.append(9);
						sbMid.append((dist-21) == 0 ? 0 : (dist-21));
					}
					else if(dist>10) {
						sbMid.append(9);
						sbMid.append((dist-11) == 0 ? 0 : (dist-11));
					}
					else {
						sbMid.append(dist-1);
					}
					return sb.substring(0,index-1) + sbMid.toString() + diagonalStr.substring(skip);
				}
			}
		}
		
		return sb.toString();
	}
	
	private String diagonalAfterWaypoint(String str) {
		int index = str.indexOf("!");
		StringBuilder sb = new StringBuilder();
		boolean diagonalFound = false;
		int wpX = waypoint.getRow();
		int wpY = waypoint.getCol();
		int tempArray[] = new int[3];
		int tempX = robot.getRow();
		int tempY = robot.getCol();
		RobotConstants.HEADING tempH = robot.getCurrentHeading();
		int j=0;
		for(int i=0; i<index; i++) {
			tempArray = this.newCoordinatesHeadingFromString(tempX, tempY, tempH, str.substring(i, i+1));
			tempX = tempArray[0];
			tempY = tempArray[1];
			tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
		}
		RobotConstants.HEADING wpH = tempH;
		if(!(wpX == tempX && wpY == tempY))
			return str.substring(index+1);
		for(int i=index+1; i<str.length(); i++) {
			if (diagonalFound)
				break;
			int skip = this.numCharToSkipIfFront(str.substring(i));
			if(skip == 0) { // char is 'R' or 'L'
				tempArray = this.newCoordinatesHeadingFromString(tempX, tempY, tempH, str.substring(i, i+1));
				tempX = tempArray[0];
				tempY = tempArray[1];
				tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
				sb.append(str.substring(i,i+1));
			}
			else {
				int checkX = tempX;
				int checkY = tempY;
				RobotConstants.HEADING checkH = tempH;
				int dist = this.goFrontDist(str.substring(i, i+skip));
				for(j=0; j<=dist; j++) {
					tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, String.valueOf(j));
					checkX = tempArray[0];
					checkY = tempArray[1];
					checkH = RobotConstants.HEADING.fromInteger(tempArray[2]);
					if(this.checkDiagonalPathNoObstacle(checkX, checkY, 13, 1)) {
						System.out.println("Diagonal from: ("+checkX+","+checkY+")");
						if(j==0) {
						}
						else if(j>20) {
							sb.append(9);
							sb.append(9);
							sb.append((j-21) == 0 ? 0 : (j-21));
						}
						else if(j>10) {
							sb.append(9);
							sb.append((j-11) == 0 ? 0 : (j-11));
						}
						else {
							sb.append(j-1);
						}
						sb.append(this.diagonalFastestTurningStr(checkX, checkY, checkH, 13, 1, RobotConstants.HEADING.NORTH));
						diagonalFound = true;
						break;
					}
				}
				if(j==dist+1) {
					tempArray = this.newCoordinatesHeadingFromString(tempX, tempY, tempH, str.substring(i, i+1));
					tempX = tempArray[0];
					tempY = tempArray[1];
					tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
					sb.append(str.substring(i, i+skip));
					i+=skip-1;
				}
			}
			System.out.println(sb.toString());
		}
		return sb.toString();
	}
	
	/*
	 * For generation of "fake" sensor string values generated from a given arena map.
	 */
	private String readSimulatorMap() {
		// TODO Auto-generated method stub
		StringBuilder sb = new StringBuilder();
		//ArrayList <Sensor> sensors = robot.getSensors();
//		if(real_map!=null) {
//			sb.append("LLR:");
//			sb.append(robot.LLR.senseReal(real_map));
//			sb.append(":LSR:");
//			sb.append(robot.LSR.senseReal(real_map));
//			sb.append(":FSR1:");
//			sb.append(robot.FSR1.senseReal(real_map));
//			sb.append(":FSR2:");
//			sb.append(robot.FSR2.senseReal(real_map));
//			sb.append(":FSR3:");
//			sb.append(robot.FSR3.senseReal(real_map));
//			sb.append(":RSR:");
//			sb.append(robot.RSR.senseReal(real_map));
//			sb.append(":");
//			if(robot.FSR1.senseImage(real_map)!=-1)
//				sb.append("1:2");
//			else if(robot.FSR2.senseImage(real_map)!=-1)
//				sb.append("2:2");
//			else if(robot.FSR3.senseImage(real_map)!=-1)
//				sb.append("3:3");
//			else
//				sb.append("-1:-1");
//			//System.out.println(sb.toString());
			
		if(real_map!=null) {
			sb.append("SL:");
			sb.append(robot.SL.senseReal(real_map));
			sb.append(":SFL:");
			sb.append(robot.SFL.senseReal(real_map));
			sb.append(":SFM:");
			sb.append(robot.SFM.senseReal(real_map));
			sb.append(":SFR:");
			sb.append(robot.SFR.senseReal(real_map));
			sb.append(":SR1:");
			sb.append(robot.SR1.senseReal(real_map));
			sb.append(":SR2:");
			sb.append(robot.SR2.senseReal(real_map));
			sb.append(":");
			if(robot.SFL.senseImage(real_map)!=-1)
				sb.append("1:2");
			else if(robot.SFM.senseImage(real_map)!=-1)
				sb.append("2:2");
			else if(robot.SFR.senseImage(real_map)!=-1)
				sb.append("3:3");
			else
				sb.append("-1:-1");
			//System.out.println(sb.toString());
		}
		
		return sb.toString();
	}

	/*
	 * Check if start
	 */
	private boolean checkIfStart() {
    	if(robot.getRow()==robot.getStartPosX() && robot.getCol() == robot.getStartPosY())
    		return true;
    	return false;
	}
	
	/*
	 * Checks for area explored
	 */
    private int calculateAreaExplored() {
        int result = 0;
        for (int r = 0; r < MapConstants.MAP_ROWS; r++)
            for (int c = 0; c < MapConstants.MAP_COLS; c++)
                if (explored_map.getTile(r, c).getIsExplored())
                    result++;
        return result;
    }
    
    /*
     * Generates MDF String
     */
	public String updateMap() {
		//System.out.println("Still here");
		String[] mdp = MapDescriptor.getMapDescriptor().generateHexadecimalEqvl(this.explored_map);
		String mdfString = mdp[0]+":"+mdp[1];
		System.out.println(mdfString);
		return  mdfString;
	}
	
	/*
	 * Check right to see if should turn right.
	 * Conditions:
	 * -> Turn right if right has unexplored tile/ free of obstacles.
	 */
	public int checkR() {
		switch(robot.getCurrentHeading()) {
		case NORTH:
			return robot.checkRight(explored_map) == -1 && checkSurrounding(cur_row+2,cur_col) ==-1 && checkSurrounding(cur_row+2,cur_col+1) ==-1 ? -1 : 1;
		case WEST:
			return robot.checkRight(explored_map) == -1 && checkSurrounding(cur_row+1,cur_col-2) ==-1 && checkSurrounding(cur_row,cur_col-2) ==-1 ? -1 : 1;
		case EAST:
			return robot.checkRight(explored_map) == -1 && checkSurrounding(cur_row,cur_col+2) ==-1 && checkSurrounding(cur_row-1,cur_col+2) ==-1 ? -1 : 1;
		case SOUTH:
			return robot.checkRight(explored_map) == -1 && checkSurrounding(cur_row-2,cur_col) ==-1 && checkSurrounding(cur_row-2,cur_col-1) ==-1 ? -1 : 1;	
		default:
			break;
		}
		return 1;
	}
	
	/*
	 * Checks right if can calibrate
	 * Returns -1 when none of the function detects obstacle/ Wall
	 * Requires 3 lined obstacle/ wall for calibration
	 */
	public int checkRCalibrate() {
		switch(robot.getCurrentHeading()) {
		case NORTH:
			return robot.checkRight(explored_map) != -1 && (checkSurrounding(cur_row+2,cur_col) !=-1 ) && (checkSurrounding(cur_row+2,cur_col+1) !=-1 ) ? -1 : 1;
		case WEST:
			return robot.checkRight(explored_map) != -1 && (checkSurrounding(cur_row+1,cur_col-2) !=-1) && checkSurrounding(cur_row,cur_col-2) !=-1 ? -1 : 1;
		case EAST:
			return robot.checkRight(explored_map) != -1 && checkSurrounding(cur_row,cur_col+2) != -1 && checkSurrounding(cur_row-1,cur_col+2) != -1 ? -1 : 1;
		case SOUTH:
			return robot.checkRight(explored_map) != -1 && checkSurrounding(cur_row-2,cur_col) != -1 && checkSurrounding(cur_row-2,cur_col-1) != -1 ? -1 : 1;	
		default:
			break;
		}
		return 1;
	}
	/*
	 * Checks for obstacles only on right only (returns 2)
	 */
	public int checkRObs() {
		int distance_center=0;
		switch(robot.getCurrentHeading()) {
		case NORTH:
			return this.checkSurrounding(cur_row+2, cur_col+1);//robot.checkRight(explored_map) != -1 && (checkSurrounding(cur_row+2,cur_col) !=-1 ) && (checkSurrounding(cur_row+2,cur_col+1) !=-1 ) ? -1 : 1;
		case WEST:
			return this.checkSurrounding(cur_row+1, cur_col-2);//obot.checkRight(explored_map) != -1 && (checkSurrounding(cur_row+1,cur_col-2) !=-1) && checkSurrounding(cur_row,cur_col-2) !=-1 ? -1 : 1;
		case EAST:
			return this.checkSurrounding(cur_row-1, cur_col+2);  //robot.checkRight(explored_map) != -1 && checkSurrounding(cur_row,cur_col+2) != -1 && checkSurrounding(cur_row-1,cur_col+2) != -1 ? -1 : 1;
		case SOUTH:
			return this.checkSurrounding(cur_row-2,  cur_col-1);//robot.checkRight(explored_map) != -1 && checkSurrounding(cur_row-2,cur_col) != -1 && checkSurrounding(cur_row-2,cur_col-1) != -1 ? -1 : 1;	
		default:
			break;
		}
		return 1;
	}
	
	/*
	 * Same as Right but on left
	 */
	public int checkL() {
		switch(robot.getCurrentHeading()) {
		case NORTH:
			return robot.checkLeft(explored_map) == -1 && checkSurrounding(cur_row-2,cur_col) ==-1 && checkSurrounding(cur_row-2,cur_col+1) ==-1 ? -1 : 1;
		case WEST:
			return robot.checkLeft(explored_map) == -1 && checkSurrounding(cur_row,cur_col+2) ==-1 && checkSurrounding(cur_row+1,cur_col+2) ==-1 ? -1 : 1;
		case EAST:
			return robot.checkLeft(explored_map) == -1 && checkSurrounding(cur_row,cur_col-2) ==-1 && checkSurrounding(cur_row-1,cur_col-2) ==-1 ? -1 : 1;
		case SOUTH:
			return robot.checkLeft(explored_map) == -1 && checkSurrounding(cur_row+2,cur_col) ==-1 && checkSurrounding(cur_row+2,cur_col-1) ==-1 ? -1 : 1;	
		default:
			break;
		}
		return 1;
	}
	
	/*
	 * Checks if all 3 front sensor detects free/ obstacle
	 */
	public int checkFront() {
		//return checkTrap(robot.checkFront(explored_map));
		return robot.checkFront(explored_map); //checkTrap(robot.checkFront(explored_map));
	}
	
	public int checkFrontForAllObstacles() {
		//return checkTrap(robot.checkFront(explored_map));
		return robot.checkFront(explored_map); //checkTrap(robot.checkFront(explored_map));
	}
	
	/*
	 * If returned value is 1 or 2, then it is valid for calibration
	 * 1 => Wall
	 * 2 => Obstacle only
	 */
	public int checkSurrounding(int row, int col){
		try {
		Tile tile = explored_map.getTile(row,col);
		//System.out.println(row+" "+col);
		if(!tile.getIsExplored())
			return -1;
		else if(tile.getIsExplored() && !tile.getIsObstacle())
			return -1;
		else
			return 2;
		}
		catch(ArrayIndexOutOfBoundsException e)
		{
			return 1;
		}
	}
	public boolean checkCanBurst(int row, int col, RobotConstants.HEADING h) {
//		if((row != 1 && row != 13) && (col != 1 && col != 18)) {
//			System.out.println("Don't burst!");
//			return false;
//		}
//		System.out.println("row:" + row);
//		System.out.println("col:" + col);
//		System.out.println("hdg:" + h);
		switch(h) {
		case NORTH:
			if(col<=2)
				return false;
			for(int i=-1; i<2; i++) {
				for(int j=0; j<2; j++) {
					if(!explored_map.getTile(row+i, col-2-j).getIsExplored() || explored_map.getTile(row+i, col-2-j).getIsObstacle())
					{
						return false;	
					}
				}
			}
			if(row<=12 && col>=2) {
				if(!explored_map.getTile(row+2, col).getIsObstacle() && !explored_map.getTile(row+2, col-1).getIsObstacle() && !explored_map.getTile(row+2, col-2).getIsObstacle())
					return false;
			}
			return true;
		case SOUTH:
			if(col>=17)
				return false;
			for(int i=-1; i<2; i++) {
				for(int j=0; j<2; j++) {
					if(!explored_map.getTile(row+i, col+2+j).getIsExplored() || explored_map.getTile(row+i, col+2+j).getIsObstacle())
						return false;
				}
			}
			if(row>=2 && col<=17) {
				if(!explored_map.getTile(row-2, col).getIsObstacle() && !explored_map.getTile(row-2, col+1).getIsObstacle() && !explored_map.getTile(row-2, col+2).getIsObstacle())
					return false;
			}
			return true;
		case WEST:
			if(row<=2)
				return false;
			for(int i=-1; i<2; i++) {
				for(int j=0; j<2; j++) {
					if(!explored_map.getTile(row-2-j, col+i).getIsExplored() || explored_map.getTile(row-2-j, col+i).getIsObstacle())
						return false;
				}
			}
			if(row>=2 && col>=2) {
				if(!explored_map.getTile(row, col-2).getIsObstacle() && !explored_map.getTile(row-1, col-2).getIsObstacle() && !explored_map.getTile(row-2, col-2).getIsObstacle())
					return false;
			}
			return true;
		case EAST:
			if(row>=12)
				return false;
			for(int i=-1; i<2; i++) {
				for(int j=0; j<2; j++) {
					if(!explored_map.getTile(row+2+j, col+i).getIsExplored() || explored_map.getTile(row+2+j, col+i).getIsObstacle())
						return false;
				}
			}
			if(row<=12 && col<=17) {
				if(!explored_map.getTile(row, col+2).getIsObstacle() && !explored_map.getTile(row+1, col+2).getIsObstacle() && !explored_map.getTile(row+2, col+2).getIsObstacle())
					return false;
			}
			return true;
		default:
			return false;
		}
	}
	public void moveRobot(boolean ignoreSensor, boolean exploration) {
		//map descriptor
		String mdfString = this.updateMap();
		//ConnectionManager.getConnMgr().sendMessage(mdfString, "AND");
		
		if(!robot.getRealRun()) {
		try {
			Thread.sleep(ROBOT_SPEED);
		}catch(InterruptedException ex) {
			Thread.currentThread().interrupt();
		}		
		//this.updateMain();
		}
		System.out.println("Sending forward command to ARD");
		//String msg = "";
		System.out.println("moveCount: " + moveCount);
		//boolean canBurst = exploration ? !processImage && this.checkCanBurst(robot.getRow(), robot.getCol(), robot.getCurrentHeading()) && moveCount<7 : false;
		//boolean canBurst = exploration ? !processImage && strcc<6 && this.checkCanBurst(robot.getRow(), robot.getCol(), robot.getCurrentHeading()) : false;
		boolean canBurst = false;
		if(robot.getRealRun()) {
			// CHANGE HERE

			if(canBurst) {
				if(moveCount<2) //moveCount<4
					ConnectionManager.getConnMgr().sendMessage("D", "ARD");
				else {
					ConnectionManager.getConnMgr().sendMessage("Z", "ARD");
				}
			}
			else {
				if(moveCount == 1 || moveCount == 3) //caliRight while moving Front
					ConnectionManager.getConnMgr().sendMessage("T", "ARD");
				else
					ConnectionManager.getConnMgr().sendMessage("F", "ARD");
			}
			//ConnectionManager.getConnMgr().sendMessage(mdfString+":"+"F", "AND");
			if(processImage) {
				//delay to prevent message concatenation
				try {
					Thread.sleep(900);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
			int tempRow = robot.getRow();
			int tempCol = robot.getCol();
			RobotConstants.HEADING tempH = robot.getCurrentHeading();
			if(this.imgToSendAnd=="")
				ConnectionManager.getConnMgr().sendMessage(mdfString+":"+tempRow+":"+(Math.abs(tempCol-MapConstants.MAP_COLS)-1)+":"+tempH.toString().charAt(0), "AND");
			else
				ConnectionManager.getConnMgr().sendMessage(mdfString+":"+tempRow+":"+(Math.abs(tempCol-MapConstants.MAP_COLS)-1)+":"+tempH.toString().charAt(0)+":"+this.imgToSendAnd, "AND");
			
		}
		if(canBurst) {
			//robot.moveRobotBurst(robot.getCurrentHeading(), explored_map);
			robot.moveRobot(robot.getCurrentHeading(),explored_map);
			this.processSensor(ignoreSensor);
			robot.moveRobot(robot.getCurrentHeading(),explored_map);
		}
		else
			robot.moveRobot(robot.getCurrentHeading(),explored_map);
		this.cur_row = robot.getRow();
		this.cur_col = robot.getCol();
		//else
			//System.out.println("Sending "+mdfString+":F to AND");
		this.processSensor(ignoreSensor); // CHRIS
		if(canBurst) {
			//this.moveCount += 2;
			this.moveCount++;
		}
		else {
			this.moveCount++;
		}
		if((moveCount>=caliThreshold && this.checkRCalibrate()==-1) || 
				(robot.getRow() == 10 && robot.getRow() == 18 && explored_map.getTile(12, 17).getIsObstacle() 
				&& !explored_map.getTile(12, 19).getIsObstacle() && !explored_map.getTile(13, 19).getIsObstacle()) ||
				(robot.getRow() == 4 && robot.getRow() == 1 && explored_map.getTile(2, 2).getIsObstacle() 
				&& !explored_map.getTile(2, 0).getIsObstacle() && !explored_map.getTile(1, 0).getIsObstacle())) {
			System.out.println("Starting to CALIBRATE RIGHT");
			//this.turnRobot('R');
			if(robot.getRealRun()) {
				//ConnectionManager.getConnMgr().sendMessage("A", "ARD");
				this.calibrateRobot('A', false);
				strcc=0;
				this.moveCount=0;
			}
			else {
				this.turnRobot('R',false);
				this.turnRobot('L',false);
				this.moveCount=0;
			}
		}
		else if((moveCount>=caliThreshold && this.checkLCalibrate()==-1) || this.checkLeftCaliWithFrontObstacle()==-1) {
			System.out.println("Starting to CALIBRATE LEFT");
			//this.turnRobot('R');
			if(robot.getRealRun()) {
				//ConnectionManager.getConnMgr().sendMessage("B", "ARD");
				this.calibrateRobot('B', false);
				strcc=0;
				this.moveCount=0;
			}
			else {
				this.turnRobot('L',false);
				this.turnRobot('R',false);
				this.moveCount=0;
			}
		}
		
//		if(this.checkFront()!=-1) {
//			moveCount=0;
//		}	
	}

	public void turnRobot(char c, boolean ignoreSensor) {	
		//map descriptor
		String mdfString = this.updateMap();
		//ConnectionManager.getConnMgr().sendMessage(mdfString, "AND");
		
		if(!robot.getRealRun())
			try {
				Thread.sleep(ROBOT_SPEED);
			}catch(InterruptedException ex) {
				Thread.currentThread().interrupt();
		}
		

		if(c=='R') {
			robot.turningRobot(c,explored_map);
		}
		else if(c=='L') {
			robot.turningRobot(c,explored_map);
		}
		else
			System.out.println("Error");
		
		//this.updateMain();
		
		//System.out.println("Sending "+c+" to ARD");
		
		if(robot.getRealRun()) {
			//System.out.println("Sending "+String.valueOf(c)+":"+mdfString+" to AND");
			//ConnectionManager.getConnMgr().sendMessage(mdfString+":"+String.valueOf(c), "AND");	
			ConnectionManager.getConnMgr().sendMessage(String.valueOf(c), "ARD");
			//ConnectionManager.getConnMgr().sendMessage(mdfString+":"+String.valueOf(c), "AND");
			if(processImage) {
				//delay to prevent message concatenation
				try {
					Thread.sleep(900);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
			if(!ignoreSensor) {
				int tempRow = robot.getRow();
				int tempCol = robot.getCol();
				RobotConstants.HEADING tempH = robot.getCurrentHeading();
				if(this.imgToSendAnd=="")
				ConnectionManager.getConnMgr().sendMessage(mdfString+":"+tempRow+":"+(Math.abs(tempCol-MapConstants.MAP_COLS)-1)+":"+tempH.toString().charAt(0), "AND");
				else
					ConnectionManager.getConnMgr().sendMessage(mdfString+":"+tempRow+":"+(Math.abs(tempCol-MapConstants.MAP_COLS)-1)+":"+tempH.toString().charAt(0)+":"+this.imgToSendAnd, "AND");
			}
			
		}//else
			//System.out.println("Sending "+mdfString+":"+String.valueOf(c)+" to AND");
		this.processSensor(ignoreSensor);

	}
	
	public void calibrateRobot(char c, boolean ignoreSensor) {	
		//map descriptor
		String mdfString = this.updateMap();
		//ConnectionManager.getConnMgr().sendMessage(mdfString, "AND");
		
		if(!robot.getRealRun())
			try {
				Thread.sleep(ROBOT_SPEED);
			}catch(InterruptedException ex) {
				Thread.currentThread().interrupt();
		}
		

		if(c=='A') {
			robot.turningRobot(c,explored_map);
		}
		else if(c=='B') {
			robot.turningRobot(c,explored_map);
		}
		else
			System.out.println("Error");
		
		//this.updateMain();
		
		//System.out.println("Sending "+c+" to ARD");
		
		if(robot.getRealRun()) {
			//System.out.println("Sending "+String.valueOf(c)+":"+mdfString+" to AND");
			//ConnectionManager.getConnMgr().sendMessage(mdfString+":"+String.valueOf(c), "AND");	
			ConnectionManager.getConnMgr().sendMessage(String.valueOf(c), "ARD");
			//ConnectionManager.getConnMgr().sendMessage(mdfString+":"+String.valueOf(c), "AND");
			if(processImage) {
				//delay to prevent message concatenation
				try {
					Thread.sleep(900);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
			if(!ignoreSensor) {
				int tempRow = robot.getRow();
				int tempCol = robot.getCol();
				RobotConstants.HEADING tempH = robot.getCurrentHeading();
				if(this.imgToSendAnd=="")
				ConnectionManager.getConnMgr().sendMessage(mdfString+":"+tempRow+":"+(Math.abs(tempCol-MapConstants.MAP_COLS)-1)+":"+tempH.toString().charAt(0), "AND");
				else
					ConnectionManager.getConnMgr().sendMessage(mdfString+":"+tempRow+":"+(Math.abs(tempCol-MapConstants.MAP_COLS)-1)+":"+tempH.toString().charAt(0)+":"+this.imgToSendAnd, "AND");
			}
			
		}//else
			//System.out.println("Sending "+mdfString+":"+String.valueOf(c)+" to AND");
		this.processSensor(ignoreSensor);
		
		if(c=='A') {
			robot.turningRobot('L',explored_map);
		}
		else if(c=='B') {
			robot.turningRobot('R',explored_map);
		}

	}
	
	/*
	 * For when camera is front, gets coordinate of front image
	 */
	private String getCoordinate(int image_distance,int sensor_type) {
		// TODO Auto-generated method stub
		int x_coor=0, y_coor=0, difference=0;
		switch(robot.getCurrentHeading()) {
		case NORTH:
			/*
			 * if sensor_type = FSR1:
			 * 		x_coor = robot.getRow()-1;
			 * 		y_coor = robot.getCol()-image_distance-1;
			 * if sensor_type = FSR3:
			 * 		x_coor = robot.getRow()+1;
			 * 		y_coor = robot.getCol()-image_distance-1;
			 * if sensor_type = FSR2:
			 * 		do below
			 */
			switch(sensor_type) {
			case 1: difference=-1;
			break;
			case 2: difference=0;
			break;
			case 3: difference=1;
			break;
			}
			x_coor = robot.getRow()+difference;
			y_coor = robot.getCol()-image_distance-1;
			break;
		case EAST:
			/*
			 * if sensor_type = FSR1:
			 * 		x_coor = robot.getRow()+image_distance+1
			 * 		y_coor = robot.getCol()-1;
			 * if sensor_type = FSR3:
			 * 		x_coor = robot.getRow()+image_distance+1;
			 * 		y_coor = robot.getCol()+1;
			 * if sensor_type = FSR2:
			 * 		do below
			 */
			switch(sensor_type) {
			case 1: difference=-1;
			break;
			case 2: difference=0;
			break;
			case 3: difference=1;
			break;
			}
			x_coor = robot.getRow()+image_distance+1;
			y_coor = robot.getCol()+difference;
			break;
		case WEST:
			/*
			 * if sensor_type = FSR1:
			 * 		x_coor = robot.getRow()-image_distance-1
			 * 		y_coor = robot.getCol()+1;
			 * if sensor_type = FSR3:
			 * 		x_coor = robot.getRow()-image_distance-1;
			 * 		y_coor = robot.getCol()-1;
			 * if sensor_type = FSR2:
			 * 		do below
			 */
			switch(sensor_type) {
			case 1: difference=1;
			break;
			case 2: difference=0;
			break;
			case 3: difference=-1;
			break;
			}
			x_coor = robot.getRow()-image_distance-1;
			y_coor = robot.getCol()+difference;
			break;
		case SOUTH:
			/*
			 * if sensor_type = FSR1:
			 * 		x_coor = robot.getRow()+1;
			 * 		y_coor = robot.getCol()+image_distance+1;
			 * if sensor_type = FSR3:
			 * 		x_coor = robot.getRow()-1;
			 * 		y_coor = robot.getCol()+image_distance+1;
			 * if sensor_type = FSR2:
			 * 		do below
			 */
			switch(sensor_type) {
			case 1: difference=1;
			break;
			case 2: difference=0;
			break;
			case 3: difference=-1;
			break;
			}
			x_coor = robot.getRow()+difference;
			y_coor = robot.getCol()+image_distance+1;
			break;
		}
		//y_coor = Math.abs(y_coor-MapConstants.MAP_COLS);
		String msg = x_coor+":"+y_coor;
		return msg;
	}

	/*
	 * Gets coordinate of right camera with offset
	 */
	private String getCoordinateRight() {
		// TODO Auto-generated method stub
		// DARREL
		int x_coor=0, y_coor=0, difference=1;
//		int x_coor=0, y_coor=0, difference=0;
		switch(robot.getCurrentHeading()) {
		case NORTH:
			x_coor = robot.getRow()+2;
			y_coor = robot.getCol()+difference;
			break;
		case EAST:
			x_coor = robot.getRow()-difference;
			y_coor = robot.getCol()+2;
			break;
		case WEST:
			x_coor = robot.getRow()+difference;
			y_coor = robot.getCol()-2;
			break;
		case SOUTH:
			x_coor = robot.getRow()-2;
			y_coor = robot.getCol()-difference;
			break;
		}
		if(robot.getRealRun()) {
			x_coor= x_coor+1;
			y_coor = Math.abs(y_coor-MapConstants.MAP_COLS);
		}
		String msg = x_coor+":"+y_coor;
		System.out.println("DEBUG :: Image coordinates "+msg);
		return msg;
	}
	
	/*
	 * Get list of directs from list of path
	 */
	public ArrayList<RobotConstants.DIRECTION> moveFPA(ArrayList<Tile> path, Tile tempTile, RobotConstants.HEADING h) {
		ArrayList<RobotConstants.DIRECTION> list_direction = new ArrayList<>();
		if(path == null) {
		       JOptionPane.showMessageDialog(null,
	    		          "Error: No path found!", "Error Message",
	    		          JOptionPane.ERROR_MESSAGE);
		       return null;
		}
		Tile curTile = tempTile;
		RobotConstants.HEADING curHeading = h;
		//RobotConstants.HEADING curHeading = robot.getCurrentHeading();
		for(Tile temp : path) {
			/*
			if(!ignoreCancel && SimulatorMDP.exploreWorker.isCancelled()) {
				System.out.println("Exploration is cancelled");
				displayMgt.cancel(true);
				return null;
			}*/
//			if(!fullCoverage) {
//				//this.processSensor();
//				//map descriptor
//				String mdfString = this.updateMap();
//				//ConnectionManager.getConnMgr().sendMessage(mdfString, "AND");
//			}
			
			RobotConstants.HEADING nextHeading = fpa.getNextHeading(curTile,temp.getRow(),temp.getCol(),curHeading);
			int turn = Math.abs(curHeading.ordinal() - nextHeading.ordinal());
			if(turn ==0) {
					//moveRobot();
					list_direction.add(RobotConstants.DIRECTION.FRONT);
			}
			else {
				if(turn==2) {
					//turnRobot('R');
					//turnRobot('R');
					//moveRobot();
					list_direction.add(RobotConstants.DIRECTION.RIGHT);
					list_direction.add(RobotConstants.DIRECTION.RIGHT);
					list_direction.add(RobotConstants.DIRECTION.FRONT);
				}
				else {
					switch(curHeading) {
					case NORTH: {
						if(curTile.getRow()-temp.getRow()>0) {
							//turnRobot('L');
							//moveRobot();
							list_direction.add(RobotConstants.DIRECTION.LEFT);
							list_direction.add(RobotConstants.DIRECTION.FRONT);
						}

						else {
							//turnRobot('R');
							//moveRobot();
							list_direction.add(RobotConstants.DIRECTION.RIGHT);
							list_direction.add(RobotConstants.DIRECTION.FRONT);
						}
						break;
					}
					case WEST: {
						if(curTile.getCol()-temp.getCol()<0) {
							//turnRobot('L');
							//moveRobot();
							list_direction.add(RobotConstants.DIRECTION.LEFT);
							list_direction.add(RobotConstants.DIRECTION.FRONT);
							
						}

						else {
							//turnRobot('R');
							//moveRobot();
							
							list_direction.add(RobotConstants.DIRECTION.RIGHT);
							list_direction.add(RobotConstants.DIRECTION.FRONT);
						}
						break;
					}
					case EAST: {
						if(curTile.getCol()-temp.getCol()>0) {
							//turnRobot('L');
							//moveRobot();
							
							list_direction.add(RobotConstants.DIRECTION.LEFT);
							list_direction.add(RobotConstants.DIRECTION.FRONT);
						}

						else {
							//turnRobot('R');
							//moveRobot();
							
							list_direction.add(RobotConstants.DIRECTION.RIGHT);
							list_direction.add(RobotConstants.DIRECTION.FRONT);
						}
						break;
					}
					case SOUTH: {
						if(curTile.getRow()-temp.getRow()<0) {
							//turnRobot('L');
							//moveRobot();
							
							list_direction.add(RobotConstants.DIRECTION.LEFT);
							list_direction.add(RobotConstants.DIRECTION.FRONT);
						}

						else {
							//turnRobot('R');
							//moveRobot();
							
							list_direction.add(RobotConstants.DIRECTION.RIGHT);
							list_direction.add(RobotConstants.DIRECTION.FRONT);
						}
						break;
					}
					}
				}
			}
			curHeading = nextHeading; //robot.getCurrentHeading();
			curTile = temp;
		}
		return list_direction;
	}
	
	public void debug_moveRobot() {
		String 	msg = this.readSimulatorMap();
		this.robot.sense(explored_map, msg);
		moveRobot(false, false);
		msg = this.readSimulatorMap();
		this.robot.sense(explored_map, msg);
	}
	
	public void debug_turnRobotRight() {
		String 	msg = this.readSimulatorMap();
		this.robot.sense(explored_map, msg);
		turnRobot('R',false);
		msg = this.readSimulatorMap();
		this.robot.sense(explored_map, msg);
	}
	
	/*
	 * Prorcesses sensor information gotten from Arduino
	 */
	private void processSensor(boolean ignoreSensor) {
		while(true) {
			String msg;
			if(robot.getRealRun()) 
				msg = ConnectionManager.getConnMgr().receiveMessage();
			else
				msg = this.readSimulatorMap();
			if(msg!=null) {
				if(!ignoreSensor) {
					int sensorIdTemp, imageIdTemp;
					System.out.println("("+LocalTime.now()+") Waiting for Sensor Information from ARD .....");
					robot.sense(explored_map,msg);
					System.out.println("("+LocalTime.now()+") Done processing Sensor Information .....");
					String updateMsg = null;
	
					updateMsg = LoopCount+". "+robot.getCurrentHeading()+" (X."+robot.getRow()+" Y."+robot.getCol()+") --- "+msg+"\n";
					String originalMsg = Main.SimulatorMDP.lblSensorInfo.getText();
					Main.SimulatorMDP.lblSensorInfo.setText(updateMsg + originalMsg);//LoopCount+". "+robot.getCurrentHeading()+" (X."+robot.getRow()+" Y."+robot.getCol()+") --- "+msg+"\n"+originalMsg);
					LoopCount++;
					//Main.SimulatorMDP.lblSensorInfo.setText(msg);
				}
				break;
			}
		}
		
		/*
		 * Finish sensor processing, ->check right for obstacle. If obstacle detected, send command to take photo!
		 */
		if(this.processImage) {
			if(this.checkRObs()==2) {
				System.out.println("Current Postion -> X:"+robot.getRow() +" Y:"+robot.getCol());
				System.out.println("Obstacle Detected On Right! ");
				this.processObstacle();
				try {
					this.sendTakePhotoCmd(); //send command to take photo
				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} 
			}
		}
		
	}
	/*
	 * Process obstacle surface, depending on sides if already have seen that surface. 
	 * e.g. South of an obstacle block. Mainly for island points
	 */
	private void processObstacle() {
		// TODO Auto-generated method stub
		String msg = this.getCoordinateRight();
		String[] sb = msg.split(":");
		int r = Integer.parseInt(sb[0]);
		int c = Integer.parseInt(sb[1]);
		if(!(r >14 || r<0 || c>19 ||c<0)) {
			Tile temp = explored_map.getTile(r, c);
			if(temp.getIsObstacle()) {
				int nextOrdinal = Math.abs(robot.getCurrentHeading().ordinal()+1);
				nextOrdinal = nextOrdinal%4;
				RobotConstants.HEADING h = RobotConstants.HEADING.values()[nextOrdinal];
				temp.replaceValue(h.toString(), true);
			}
		}
	}
	
	/*
	 * Set and Replacing Waypoints set
	 */
	public void setAndReplaceWP(int xRow, int yCol) {
		if(!robot.getRealRun())
			real_map.setWaypoint(xRow, yCol);
		explored_map.setWaypoint(xRow, yCol);
		
	}
	
	public int checkTrap(int frontValue) {
		if(frontValue == -1)
			return -1;
		else {
			if(frontValue == -2) {
				if(this.checkR()!=-1 && this.checkL()!=-1) {
					return 1;
				}
				else
					return -1;
			}
		}
		return 1;
	}
	/* Traps Detection
	public int checkIfTrap() {
		//if(this.lookFront2IfWall()==1)
			//return this.check2IfTrap();
		if(lookFrontIfWall()==-1)
			return -1;

		
		return this.trap1();// || this.trap2()==1 ? 1 : -1;
	}
	
	private int trap2() {
		switch(robot.getCurrentHeading()) {
		case EAST:
		return (checkSurrounding(cur_row,cur_col-3) == 1 && checkSurrounding(cur_row+1,cur_col-3) == 1 && checkSurrounding(cur_row-1, cur_col-3) == 1) && 
		 (checkSurrounding(cur_row,cur_col+2) == 1 && checkSurrounding(cur_row+1,cur_col+2)==1 && checkSurrounding(cur_row-1, cur_col+2)==1) ? 1:-1;
		case WEST:
		return (checkSurrounding(cur_row,cur_col-3) == 1 && checkSurrounding(cur_row+1,cur_col-3) == 1 && checkSurrounding(cur_row-1, cur_col-3) == 1) && 
		 (checkSurrounding(cur_row,cur_col+2) == 1 && checkSurrounding(cur_row+1,cur_col+2)==1 && checkSurrounding(cur_row-1, cur_col+2)==1) ? 1:-1;
		case NORTH:
		return (checkSurrounding(cur_row-3,cur_col) == 1 && checkSurrounding(cur_row-3,cur_col+1) == 1 && checkSurrounding(cur_row-3, cur_col-1) == 1) && 
		 (checkSurrounding(cur_row+2,cur_col) == 1 && checkSurrounding(cur_row+2,cur_col+1)==1 && checkSurrounding(cur_row+2, cur_col-1)==1) ? 1:-1;
		case SOUTH:
			return (checkSurrounding(cur_row+3,cur_col) == 1 && checkSurrounding(cur_row+3,cur_col+1) == 1 && checkSurrounding(cur_row+3, cur_col-1) == 1) && 
					 (checkSurrounding(cur_row-2,cur_col) == 1 && checkSurrounding(cur_row-2,cur_col+1)==1 && checkSurrounding(cur_row-2, cur_col-1)==1) ? 1:-1;
	
	}
	return -1;
	}

	public int trap1() {
		switch(robot.getCurrentHeading()) {
		case EAST:
		return (checkSurrounding(cur_row,cur_col-3) == 2 && checkSurrounding(cur_row+1,cur_col-3) == 2 && checkSurrounding(cur_row-1, cur_col-3) == 2) && 
		 (checkSurrounding(cur_row,cur_col+3) == 2 && checkSurrounding(cur_row+1,cur_col+3)==2 && checkSurrounding(cur_row-1, cur_col+3)==2) ? 1:-1;
		case WEST:
		return (checkSurrounding(cur_row,cur_col-3) == 2 && checkSurrounding(cur_row+1,cur_col-3) == 2 && checkSurrounding(cur_row-1, cur_col-3) == 2) && 
		 (checkSurrounding(cur_row,cur_col+3) == 2 && checkSurrounding(cur_row+1,cur_col+3)==2 && checkSurrounding(cur_row-1, cur_col+3)==2) ? 1:-1;
		case NORTH:
		return (checkSurrounding(cur_row-3,cur_col) == 2 && checkSurrounding(cur_row-3,cur_col+1) == 2 && checkSurrounding(cur_row-3, cur_col-1) == 2) && 
		 (checkSurrounding(cur_row+3,cur_col) == 2 && checkSurrounding(cur_row+3,cur_col+1)==2 && checkSurrounding(cur_row+3, cur_col-1)==2) ? 1:-1;
		case SOUTH:
			return (checkSurrounding(cur_row+3,cur_col) == 2 && checkSurrounding(cur_row+3,cur_col+1) == 2 && checkSurrounding(cur_row+3, cur_col-1) == 2) && 
					 (checkSurrounding(cur_row-3,cur_col) == 2 && checkSurrounding(cur_row-3,cur_col+1)==2 && checkSurrounding(cur_row-3, cur_col-1)==2) ? 1:-1;
	
	}
	return -1;
	}
	public int check2IfTrap() {
		switch(robot.getCurrentHeading()) {
			case EAST:
			return (checkSurrounding(cur_row,cur_col-3) == 1 && checkSurrounding(cur_row+1,cur_col-3) == 1 && checkSurrounding(cur_row-1, cur_col-3) == 1) && 
			 (checkSurrounding(cur_row,cur_col+3) == 1 && checkSurrounding(cur_row+1,cur_col+3)==1 && checkSurrounding(cur_row-1, cur_col+3)==1) ? 2:-1;
			case WEST:
			return (checkSurrounding(cur_row,cur_col-3) == 1 && checkSurrounding(cur_row+1,cur_col-3) == 1 && checkSurrounding(cur_row-1, cur_col-3) == 1) && 
			 (checkSurrounding(cur_row,cur_col+3) == 1 && checkSurrounding(cur_row+1,cur_col+3)==1 && checkSurrounding(cur_row-1, cur_col+3)==1) ? 2:-1;
			case NORTH:
			return (checkSurrounding(cur_row-3,cur_col) == 1 && checkSurrounding(cur_row-3,cur_col+1) == 1 && checkSurrounding(cur_row-3, cur_col-1) == 1) && 
			 (checkSurrounding(cur_row+3,cur_col) == 1 && checkSurrounding(cur_row+3,cur_col+1)==1 && checkSurrounding(cur_row+3, cur_col-1)==1) ? 2:-1;
			case SOUTH:
				return (checkSurrounding(cur_row+3,cur_col) == 1 && checkSurrounding(cur_row+3,cur_col+1) == 1 && checkSurrounding(cur_row+3, cur_col-1) == 1) && 
						 (checkSurrounding(cur_row-3,cur_col) == 1 && checkSurrounding(cur_row-3,cur_col+1)==1 && checkSurrounding(cur_row-3, cur_col-1)==1) ? 2:-1;
		
		}
		return -1;
	}
	public int lookFrontIfWall() {
		switch(robot.getCurrentHeading()) {
		case NORTH:
			if((cur_col-2)<0)
				return 1;
			break;
		case SOUTH:
			if((cur_col+2)>19)
				return 1;
			break;
		case WEST:
			if((cur_row-2)<0)
				return 1;
			break;
		case EAST:
			if((cur_row+2)>14)
				return 1;
			break;
		}
		return -1;
	}
	public int lookFront2IfWall() {
		switch(robot.getCurrentHeading()) {
		case NORTH:
			if((cur_col-3)==-2)
				return 1;
			break;
		case SOUTH:
			if((cur_col+3)==20)
				return 1;
			break;
		case WEST:
			if((cur_row-3)==-2)
				return 1;
			break;
		case EAST:
			if((cur_row+3)==15)
				return 1;
			break;
		}
		return -1;
	}

	public int lookFrontAhead() {
		switch(robot.getCurrentHeading()) {
		case NORTH:
			return checkSurrounding(cur_row,cur_col-3) ==1 && checkSurrounding(cur_row-1,cur_col-3)==1 && checkSurrounding(cur_row+1, cur_col-3)==1 ? 1:-1;
		case EAST:
			return checkSurrounding(cur_row+3,cur_col) ==1 && checkSurrounding(cur_row+3,cur_col-1)==1 && checkSurrounding(cur_row+3, cur_col+1)==1 ? 1:-1;
		case WEST:
			return checkSurrounding(cur_row-3,cur_col) ==1 && checkSurrounding(cur_row-3,cur_col-1)==1 && checkSurrounding(cur_row-3, cur_col+1)==1 ? 1:-1;
		case SOUTH:
			return checkSurrounding(cur_row,cur_col+3) ==1 && checkSurrounding(cur_row-1,cur_col+3)==1 && checkSurrounding(cur_row+1, cur_col+3)==1 ? 1:-1;
				
		default:
			break;
		}
		return -1;
	}
	*/
	
	/*
	 * Right-Wall Hug Algorithm
	 * 1. Prioritize Right Turn
	 * 2. Move Forward if avail.
	 * 3. Turn left if 1 and 2 fails.
	 * 4. Else double turn
	 */
	public void rightWallHug(boolean v, Tile ref) {
		//init sensors
		//this.processSensor();
		//map descriptor
		//this.updateMap();
		
		//algorithm
		if(v) {
			if(this.checkR()==-1) {
				this.turnRobot('R',false);
				if(strcc % 2 == 0) {
					strcc++;
				}
				else {
					strcc = 0;
				}
				//System.out.println("RIGHT CLEAR CHECK");
				if(!continueRun)
					return;
				if(this.checkFront()==-1) {
					if(strcc%2!=0)
						strcc++;
					else
						strcc=0;
					this.moveRobot(false, true);
				}
				else {
					this.turnRobot('L',false);
					if(strcc % 2 == 0) {
						strcc++;
					}
					else {
						strcc = 0;
					}
					if(this.checkFront()==-1) {
						if(strcc%2!=0)
							strcc++;
						else
							strcc=0;
						this.moveRobot(false, true);
					}
				}
			}
			else if(this.checkFront()==-1) {
				if(strcc%2!=0)
					strcc++;
				else
					strcc=0;
				this.moveRobot(false, true);
			}
			else if(this.checkL()==-1) {
				this.turnRobot('L',false);
				if(strcc % 2 == 0) {
					strcc++;
				}
				else {
					strcc = 0;
				}
			}
			else{
				this.turnRobot('L',false);
				if(!continueRun)
					return;
				this.turnRobot('L',false);
			}
			
			this.cur_row = robot.getRow();
			this.cur_col = robot.getCol();
	
			explored_map.paintAgain();
			
			if(SimulatorMDP.exploreWorker.isCancelled()) {
				this.stopRun();
				return;
			}
		}
		else { //for island hugging obsolete. 
			if(this.checkR()==-1) {
				this.turnRobot('R',false);
				//System.out.println("RIGHT CLEAR CHECK");
				if(!continueRun)
					return;
				if(this.checkFront()==-1) {
					this.moveRobot(false, true);
				}
			}
			else if(this.checkFront()==-1) {
					this.moveRobot(false, true);
			}
			else if(this.checkL()==-1) {
				this.turnRobot('L',false);
			}
			else{
				this.turnRobot('L',false);
				if(!continueRun)
					return;
				this.turnRobot('L',false);
				if (this.checkRObs() != 2)
					this.leftWallHug(false, ref);
			}
			
			this.cur_row = robot.getRow();
			this.cur_col = robot.getCol();
	
			explored_map.paintAgain();
			
		}
	}

	/*
	 * Algorithm for left wall hug; same as RWH, just opposite.
	 */
	public void leftWallHug(boolean v, Tile ref) {
		//init sensors
		//this.processSensor();
		//map descriptor
		//this.updateMap();
		
		//algorithm
		while(true) {
		if(this.checkL()==-1) {
			this.turnRobot('L',false);
			//System.out.println("RIGHT CLEAR CHECK");
			if(!continueRun)
				return;
			if(this.checkFront()==-1) {
				this.moveRobot(false, true);
			}
		}
		else if(this.checkFront()==-1) {
			this.moveRobot(false, true);
		}
		else if(this.checkR()==-1) {
			this.turnRobot('R',false);
		}
		else {
			this.turnRobot('R',false);
			if(!continueRun)
				return;
			this.turnRobot('R',false);
		}
		this.cur_row = robot.getRow();
		this.cur_col = robot.getCol();

		explored_map.paintAgain();
		
		if(cur_row == ref.getRow() && cur_col==ref.getCol())
		return;
		}
	}
	
	/*
	 * Algorithm for sending take photo command
	 * If Right has obstacle -> chance for image -> send message to RPI to take photo
	 */
	private void sendTakePhotoCmd() throws IOException {
		String msg="";
		debug_photoCmdCount++;
		picture_count++;
		System.out.println("Sending Photo Command to RPI");
		String Imagloc = this.getCoordinateRight();
		if(robot.getRealRun()) {
//			ConnectionManager.getConnMgr().sendMessage("P", "RPI");
//			try {
//				Thread.sleep(800);
//			} catch (InterruptedException e) {
//				// TODO Auto-generated catch block
//				e.printStackTrace();
//			}
			ConnectionManager.getConnMgr().sendMessage(Imagloc, "RPI");
			try {
				Thread.sleep(900);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			msg = "";
		}
		else {
			String sb = this.getCoordinateRight();
			String[] coor = sb.split(":");
			int y_coor = Integer.parseInt(coor[1]);
			int x_coor = Integer.parseInt(coor[0]);
			Tile temp;
			if((temp=explored_map.getTile(x_coor, y_coor)).getIsImage()) {
				msg=temp.getImage().getId();
			}
			else
				msg="None";
		}
		if(!msg.equalsIgnoreCase("None"))
			this.processImageReceive("1:1", msg);
	}
	
	/*
	 * Process image received, if any, from RPI with respective coordinates and the Photo ID.
	 */
	private void processImageReceive(String coordinate, String imageMsg) {
		System.out.println("Called the function to process the image!!!");
		if (imageMsg != ""){
			// TODO Auto-generated method stub
			System.out.println("Debug :: Processing Image Received!");
			//imageDetails = imageMsg.split(":");
			String imgId = imageMsg;
			//String sensorPos = imageDetails[1];
			//int sensorPosI = Integer.parseInt(sensorPos.substring(sensorPos.length()-1));
//		int roguePos =-1;
//		switch(sensorPosI) {
//		case 1: roguePos=-1;
//		break;
//		case 2: roguePos=0;
//		break;
//		case 3: roguePos=1;
//		break;
//
//		}
//		System.out.println("DEBUG :: Sensor Position ->"+sensorPosI);
//		System.out.println("DEBUG :: Rogue Position ->"+roguePos);
//		int image_distance=  this.findDistance(roguePos);
//		System.out.println("DEBUG :: IMAGE DISTANCE ->"+image_distance);
//		if(image_distance==-1)
//			return;
			//DARREL
			String imagemsg = coordinate + ":"+imgId; //this.getCoordinate(image_distance, sensorPosI) + ":" +imgId;
			//if(!imageIdsList.contains(imgId)) {
			System.out.println("DEBUG MSG :: FOUND IMAGE - >" +imagemsg);
			imageList.add(imagemsg);
			imageIdsList.add(imgId);
			this.imgToSendAnd = imagemsg; //sets image to send message (wait for next round of communication)
			//}
		}
	}
	
	/*
	 * Checks for Left is obstacle only for calibration
	 */
	public int checkLCalibrate() {
		switch(robot.getCurrentHeading()) {
		case NORTH:
			return checkSurrounding(cur_row-2,cur_col-1) ==2 && checkSurrounding(cur_row-2,cur_col) ==2 && checkSurrounding(cur_row-2,cur_col+1) ==2 ? -1 : 1;
		case WEST:
			return checkSurrounding(cur_row-1,cur_col+2) ==2 && checkSurrounding(cur_row,cur_col+2) ==2 && checkSurrounding(cur_row+1,cur_col+2) ==2 ? -1 : 1;
		case EAST:
			return checkSurrounding(cur_row+1,cur_col-2) ==2 && checkSurrounding(cur_row,cur_col-2) ==2 && checkSurrounding(cur_row-1,cur_col-2) ==2 ? -1 : 1;
		case SOUTH:
			return checkSurrounding(cur_row+2,cur_col+1) ==2 && checkSurrounding(cur_row+2,cur_col) ==2 && checkSurrounding(cur_row+2,cur_col-1) ==2 ? -1 : 1;	
		default:
			break;
		}
		return 1;
	} 
	
	public int checkLeftCaliWithFrontObstacle() {
		
		switch(robot.getCurrentHeading()) {
		case NORTH:
			return checkSurrounding(cur_row-2,cur_col-1) ==1 && checkSurrounding(cur_row-2,cur_col) ==1 && checkSurrounding(cur_row-2,cur_col+1) == 1
			&& checkSurrounding(cur_row-1,cur_col-2) ==2 && checkSurrounding(cur_row,cur_col-2)==2 && checkSurrounding(cur_row+1,cur_col-2) ==2 ? -1 : 1;
		case WEST:
			return checkSurrounding(cur_row-1,cur_col+2) ==1 && checkSurrounding(cur_row,cur_col+2) ==1 && checkSurrounding(cur_row+1,cur_col+2) ==1 
			&& checkSurrounding(cur_row-2,cur_col-1)==2 && checkSurrounding(cur_row-2,cur_col)==2 && checkSurrounding(cur_row-2,cur_col+1)==2 ? -1 : 1;
		case EAST:
			return checkSurrounding(cur_row+1,cur_col-2) ==1 && checkSurrounding(cur_row,cur_col-2) ==1 && checkSurrounding(cur_row-1,cur_col-2) ==1 
			&& checkSurrounding(cur_row+2,cur_col-1)==2 && checkSurrounding(cur_row+2,cur_col)==2 && checkSurrounding(cur_row+2,cur_col+1)==2 ? -1 : 1;
		case SOUTH:
			return checkSurrounding(cur_row+2,cur_col+1) ==1 && checkSurrounding(cur_row+2,cur_col) ==1 && checkSurrounding(cur_row+2,cur_col-1) ==1
			&& checkSurrounding(cur_row-1,cur_col+2)==2 && checkSurrounding(cur_row,cur_col+2)==2 && checkSurrounding(cur_row+1,cur_col+2)==2 ? -1 : 1;	
		default:
			break;
		}
		return 1;
	}
	
	public String diagonalFPString(String sb, boolean fastestPath) {
		if(!fastestPath)
			return sb;
		//double degs = toDegrees(atan(tangent));
		StringBuilder sbBackup = new StringBuilder(); // for before passing waypoint
		StringBuilder sbConfirmed = new StringBuilder(); // for after passing waypoint (CONFIRMED)
		boolean waypointCleared = true;
		boolean pathFound = false;
		int fromX = robot.getRow();
		int fromY = robot.getCol();
		RobotConstants.HEADING robotHeading = robot.getCurrentHeading();
		int tempX = robot.getRow();
		int tempY = robot.getCol();
		RobotConstants.HEADING tempH = robot.getCurrentHeading();
		int tempFromX = 0;
		int tempFromY = 0;
		RobotConstants.HEADING tempFromH = robot.getCurrentHeading();
		int endX = 0;
		int endY = 0;
		RobotConstants.HEADING endH = robot.getCurrentHeading();
		int diff = 0;
		int a;
		int[] tempArray = new int[3];
		for(int i=0; i<sb.toString().length(); i++) {
			int sum1 = 0;
			int sum2 = 0;
			int sum3 = 0;
			int sum4 = 0;
			int skip1 = 0;
			int skip2 = 0;
			int skip3 = 0;
			int skip4 = 0;
			skip1 = this.numCharToSkipIfFront(sb);
			if(skip1>0) { // forward
				sum1 = this.goFrontDist(sb.substring(i, i+skip1));
				if(i+skip1+1<=sb.length()) {
					if(sb.substring(i+skip1, i+skip1+1).compareToIgnoreCase("U") == 0) { // R
						skip2 = this.numCharToSkipIfFront(sb.substring(i+skip1+1));
						if(skip2>0) { // forward
							sum2 = this.goFrontDist(sb.substring(i+skip1+1, i+skip1+skip2+1));
							if(i+skip1+skip2+2<=sb.length()) {
								if(sb.substring(i+skip1+skip2+1, i+skip1+skip2+2).compareToIgnoreCase("L") == 0) { // L
									skip3 = this.numCharToSkipIfFront(sb.substring(i+skip1+skip2+2));
									if (skip3>0) { // forward
										sum3 = this.goFrontDist(sb.substring(i+skip1+skip2+2, i+skip1+skip2+skip3+2));
										if(i+skip1+skip2+skip3+3<=sb.length()) {
											if(sb.substring(i+skip1+skip2+skip3+2, i+skip1+skip2+skip3+3).compareToIgnoreCase("U") == 0) { // R
												skip4 = this.numCharToSkipIfFront(sb.substring(i+skip1+skip2+skip3+3));
												if(skip4>0) { // forward
													sum4 = this.goFrontDist(sb.substring(i+skip1+skip2+skip3+3, i+skip1+skip2+skip3+skip4+3));
													
													tempX = fromX;
													tempY = fromY;
													tempH = robotHeading;
													tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, String.valueOf(sum1));
													tempX = tempArray[0];
													tempY = tempArray[1];
													tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
													tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, "R");
													tempX = tempArray[0];
													tempY = tempArray[1];
													tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
													tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, String.valueOf(sum2));
													tempX = tempArray[0];
													tempY = tempArray[1];
													tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
													tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, "L");
													tempX = tempArray[0];
													tempY = tempArray[1];
													tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
													tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, String.valueOf(sum3));
													tempX = tempArray[0];
													tempY = tempArray[1];
													tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
													tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, "R");
													tempX = tempArray[0];
													tempY = tempArray[1];
													tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
													pathFound = false;
													for(a=0; a<=sum1; a++) {
														if(pathFound) {
															break;
														}
														tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(a));
														tempFromX = tempArray[0];
														tempFromY = tempArray[1];
														tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
														for(diff=sum4; diff>0; diff--) {
															tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, String.valueOf(diff));
															endX = tempArray[0];
															endY = tempArray[1];
															endH = RobotConstants.HEADING.fromInteger(tempArray[2]);
															if(this.checkDiagonalPathNoObstacle(tempFromX, tempFromY, endX, endY)) {
																if(!waypointCleared) {
																	if(this.checkDiagonalPathWaypoint(tempFromX, tempFromY, endX, endY)) {
																		sbConfirmed.append(sbBackup);
																		waypointCleared = true;
																	}
																}
																if(waypointCleared) {
																	if(a==0) {
																		System.out.println("HERE1");
																		System.out.println("Don't append.");
																	}
																	else if(a>10) {
																		for(int k=0; k<=a; k++) {
																			tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(k));
																			tempFromX = tempArray[0];
																			tempFromY = tempArray[1];
																			tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
																			if(explored_map.getTile(tempFromX, tempFromY).getIsWaypoint())
																				waypointCleared = true;
																		}
																		sbConfirmed.append(9);
																		sbConfirmed.append((a-11) == 0 ? 0 : (a-11));
																	}
																	else {
																		for(int k=0; k<=a; k++) {
																			tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(k));
																			tempFromX = tempArray[0];
																			tempFromY = tempArray[1];
																			tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
																			if(explored_map.getTile(tempFromX, tempFromY).getIsWaypoint())
																				waypointCleared = true;
																		}
																		sbConfirmed.append(a-1);
																	}
																	sbConfirmed.append(this.diagonalFastestTurningStr(tempFromX, tempFromY, tempFromH, endX, endY, endH));
																}
																else {
																	if(a==0) {
																		System.out.println("HERE2");
																		System.out.println("Don't append.");
																	}
																	else if(a>10) {
																		sbBackup.append(9);
																		sbBackup.append((a-11) == 0 ? 0 : (a-11));
																	}
																	else {
																		sbBackup.append(a-1);
																	}
																	sbBackup.append(this.diagonalFastestTurningStr(tempFromX, tempFromY, tempFromH, endX, endY, endH));
																}
																fromX = endX;
																fromY = endY;
																robotHeading = endH;
																if(diff != sum4) {
																	if(sum4-diff>10) {
																		sbBackup.append(9);
																		sbBackup.append((sum4-diff-11) == 0 ? 0 : (sum4-diff-11));
																	}
																	else {
																		sbBackup.append(sum4-diff-1);
																	}
																}
																pathFound = true;
																break;
															}
														}
													}
													if(diff != 0) {
														i += skip1+skip2+skip3+skip4+2;
													}
													else if (diff==0 && a==sum1+1){
														for(int k=0; k<=sum1; k++) {
															tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(k));
															tempFromX = tempArray[0];
															tempFromY = tempArray[1];
															tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
															if(explored_map.getTile(tempFromX, tempFromY).getIsWaypoint())
																waypointCleared = true;
														}
														fromX = tempFromX;
														fromY = tempFromY;
														robotHeading = tempFromH;
														if(waypointCleared) {
															sbConfirmed.append(sb.substring(i, i+skip1));
														}
														else {
															sbBackup.append(sb.substring(i, i+skip1));
														}
														i+=skip1-1;
													}
												}
											}
											else {
												tempX = fromX;
												tempY = fromY;
												tempH = robotHeading;
												tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, String.valueOf(sum1));
												tempX = tempArray[0];
												tempY = tempArray[1];
												tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
												tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, "R");
												tempX = tempArray[0];
												tempY = tempArray[1];
												tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
												tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, String.valueOf(sum2));
												tempX = tempArray[0];
												tempY = tempArray[1];
												tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
												tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, "L");
												tempX = tempArray[0];
												tempY = tempArray[1];
												tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
												pathFound = false;
												System.out.println("sum1: " + sum1);
												for(a=0; a<=sum1; a++) {
													if(pathFound) {
														break;
													}
													System.out.println("fromX: " + fromX);
													System.out.println("fromY: " + fromY);
													tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(a));
													tempFromX = tempArray[0];
													tempFromY = tempArray[1];
													tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
													System.out.println("sum3: " + sum3);
													for(diff=sum3; diff>0; diff--) {
														tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, String.valueOf(diff));
														endX = tempArray[0];
														endY = tempArray[1];
														endH = RobotConstants.HEADING.fromInteger(tempArray[2]);
														System.out.println("endX: " + endX);
														System.out.println("endY: " + endY);
														if(this.checkDiagonalPathNoObstacle(tempFromX, tempFromY, endX, endY)) {
															System.out.println("Diagonal Path No Obstacles!");
															if(!waypointCleared) {
																if(this.checkDiagonalPathWaypoint(tempFromX, tempFromY, endX, endY)) {
																	sbConfirmed.append(sbBackup);
																	waypointCleared = true;
																}
															}
															System.out.println("waypointCleared: " + waypointCleared);
															if(waypointCleared) {
																if(a==0) {
																	System.out.println("HERE3");
																	System.out.println("Don't append.");
																}
																else if(a>10) {
																	for(int k=0; k<=a; k++) {
																		tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(k));
																		tempFromX = tempArray[0];
																		tempFromY = tempArray[1];
																		tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
																		if(explored_map.getTile(tempFromX, tempFromY).getIsWaypoint())
																			waypointCleared = true;
																	}
																	sbConfirmed.append(9);
																	sbConfirmed.append((a-11) == 0 ? 0 : (a-11));
																}
																else {
																	for(int k=0; k<=a; k++) {
																		tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(k));
																		tempFromX = tempArray[0];
																		tempFromY = tempArray[1];
																		tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
																		if(explored_map.getTile(tempFromX, tempFromY).getIsWaypoint())
																			waypointCleared = true;
																	}
																	sbConfirmed.append(a-1);
																}
																sbConfirmed.append(this.diagonalFastestTurningStr(tempFromX, tempFromY, tempFromH, endX, endY, endH));
															}
															else {
																if(a==0) {
																	System.out.println("HERE4");
																	System.out.println("Don't append.");
																}
																else if(a>10) {
																	sbBackup.append(9);
																	sbBackup.append((a-11) == 0 ? 0 : (a-11));
																}
																else {
																	sbBackup.append(a-1);
																}
																sbBackup.append(this.diagonalFastestTurningStr(tempFromX, tempFromY, tempFromH, endX, endY, endH));
															}
															fromX = endX;
															fromY = endY;
															robotHeading = endH;
															if(diff != sum3) {
																if(sum3-diff>10) {
																	sbBackup.append(9);
																	sbBackup.append((sum3-diff-11) == 0 ? 0 : (sum3-diff-11));
																}
																else {
																	sbBackup.append(sum3-diff-1);
																}
															}
															pathFound = true;
															break;
														}
													}
												}
												if(diff != 0) {
													i += skip1+skip2+skip3+1;
												}
												else if (diff==0 && a==sum1+1){
													for(int k=0; k<=sum1; k++) {
														tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(k));
														tempFromX = tempArray[0];
														tempFromY = tempArray[1];
														tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
														if(explored_map.getTile(tempFromX, tempFromY).getIsWaypoint())
															waypointCleared = true;
													}
													fromX = tempFromX;
													fromY = tempFromY;
													robotHeading = tempFromH;
													if(waypointCleared) {
														sbConfirmed.append(sb.substring(i, i+skip1));
													}
													else {
														sbBackup.append(sb.substring(i, i+skip1));
													}
													i+=skip1-1;
												}
											}
										}
										else {
//											sbBackup.append(sb.substring(i,i+skip1+skip2+skip3+2));
//											i += skip1+skip2+skip3+1;
											tempX = fromX;
											tempY = fromY;
											tempH = robotHeading;
											tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, String.valueOf(sum1));
											tempX = tempArray[0];
											tempY = tempArray[1];
											tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
											tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, "R");
											tempX = tempArray[0];
											tempY = tempArray[1];
											tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
											tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, String.valueOf(sum2));
											tempX = tempArray[0];
											tempY = tempArray[1];
											tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
											tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, "L");
											tempX = tempArray[0];
											tempY = tempArray[1];
											tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
											pathFound = false;
											for(a=0; a<=sum1; a++) {
												if(pathFound) {
													break;
												}
												tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(a));
												tempFromX = tempArray[0];
												tempFromY = tempArray[1];
												tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
												for(diff=sum3; diff>0; diff--) {
													tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, String.valueOf(diff));
													endX = tempArray[0];
													endY = tempArray[1];
													endH = RobotConstants.HEADING.fromInteger(tempArray[2]);
													if(this.checkDiagonalPathNoObstacle(tempFromX, tempFromY, endX, endY)) {
														if(!waypointCleared) {
															if(this.checkDiagonalPathWaypoint(tempFromX, tempFromY, endX, endY)) {
																sbConfirmed.append(sbBackup);
																waypointCleared = true;
															}
														}
														if(waypointCleared) {
															if(a==0) {
																System.out.println("HERE5");
																System.out.println("Don't append.");
															}
															else if(a>10) {
																for(int k=0; k<=a; k++) {
																	tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(k));
																	tempFromX = tempArray[0];
																	tempFromY = tempArray[1];
																	tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
																	if(explored_map.getTile(tempFromX, tempFromY).getIsWaypoint())
																		waypointCleared = true;
																}
																sbConfirmed.append(9);
																sbConfirmed.append((a-11) == 0 ? 0 : (a-11));
															}
															else {
																for(int k=0; k<=a; k++) {
																	tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(k));
																	tempFromX = tempArray[0];
																	tempFromY = tempArray[1];
																	tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
																	if(explored_map.getTile(tempFromX, tempFromY).getIsWaypoint())
																		waypointCleared = true;
																}
																sbConfirmed.append(a-1);
															}
															sbConfirmed.append(this.diagonalFastestTurningStr(tempFromX, tempFromY, tempFromH, endX, endY, endH));
														}
														else {
															if(a==0) {
																System.out.println("HERE6");
																System.out.println("Don't append.");
															}
															else if(a>10) {
																sbBackup.append(9);
																sbBackup.append((a-11) == 0 ? 0 : (a-11));
															}
															else {
																sbBackup.append(a-1);
															}
															sbBackup.append(this.diagonalFastestTurningStr(tempFromX, tempFromY, tempFromH, endX, endY, endH));
														}
														fromX = endX;
														fromY = endY;
														robotHeading = endH;
														if(diff != sum3) {
															if(sum3-diff>10) {
																sbBackup.append(9);
																sbBackup.append((sum3-diff-11) == 0 ? 0 : (sum3-diff-11));
															}
															else {
																sbBackup.append(sum3-diff-1);
															}
														}
														pathFound = true;
														break;
													}
												}
											}
											if(diff != 0) {
												i += skip1+skip2+skip3+1;
											}
											else if (diff==0 && a==sum1+1){
												for(int k=0; k<=sum1; k++) {
													tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(k));
													tempFromX = tempArray[0];
													tempFromY = tempArray[1];
													tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
													if(explored_map.getTile(tempFromX, tempFromY).getIsWaypoint())
														waypointCleared = true;
												}
												fromX = tempFromX;
												fromY = tempFromY;
												robotHeading = tempFromH;
												if(waypointCleared) {
													sbConfirmed.append(sb.substring(i, i+skip1));
												}
												else {
													sbBackup.append(sb.substring(i, i+skip1));
												}
												i+=skip1-1;
											}
										}
									}
								}
								else {
									tempX = fromX;
									tempY = fromY;
									tempH = robotHeading;
									tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, String.valueOf(sum1));
									tempX = tempArray[0];
									tempY = tempArray[1];
									tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
									tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, "R");
									tempX = tempArray[0];
									tempY = tempArray[1];
									tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
									pathFound = false;
									for(a=0; a<=sum1; a++) {
										if(pathFound) {
											break;
										}
										tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(a));
										tempFromX = tempArray[0];
										tempFromY = tempArray[1];
										tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
										for(diff=sum2; diff>0; diff--) {
											tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, String.valueOf(diff));
											endX = tempArray[0];
											endY = tempArray[1];
											endH = RobotConstants.HEADING.fromInteger(tempArray[2]);
											if(this.checkDiagonalPathNoObstacle(tempFromX, tempFromY, endX, endY)) {
												if(!waypointCleared) {
													if(this.checkDiagonalPathWaypoint(tempFromX, tempFromY, endX, endY)) {
														sbConfirmed.append(sbBackup);
														waypointCleared = true;
													}
												}
												if(waypointCleared) {
													if(a==0) {
														System.out.println("HERE7");
														System.out.println("Don't append.");
													}
													else if(a>10) {
														for(int k=0; k<=a; k++) {
															tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(k));
															tempFromX = tempArray[0];
															tempFromY = tempArray[1];
															tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
															if(explored_map.getTile(tempFromX, tempFromY).getIsWaypoint())
																waypointCleared = true;
														}
														sbConfirmed.append(9);
														sbConfirmed.append((a-11) == 0 ? 0 : (a-11));
													}
													else {
														for(int k=0; k<=a; k++) {
															tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(k));
															tempFromX = tempArray[0];
															tempFromY = tempArray[1];
															tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
															if(explored_map.getTile(tempFromX, tempFromY).getIsWaypoint())
																waypointCleared = true;
														}
														sbConfirmed.append(a-1);
													}
													sbConfirmed.append(this.diagonalFastestTurningStr(tempFromX, tempFromY, tempFromH, endX, endY, endH));
												}
												else {
													if(a==0) {
														System.out.println("HERE8");
														System.out.println("Don't append.");
													}
													else if(a>10) {
														sbBackup.append(9);
														sbBackup.append((a-11) == 0 ? 0 : (a-11));
													}
													else {
														sbBackup.append(a-1);
													}
													sbBackup.append(this.diagonalFastestTurningStr(tempFromX, tempFromY, tempFromH, endX, endY, endH));
												}
												fromX = endX;
												fromY = endY;
												robotHeading = endH;
												if(diff != sum2) {
													if(sum2-diff>10) {
														sbBackup.append(9);
														sbBackup.append((sum2-diff-11) == 0 ? 0 : (sum2-diff-11));
													}
													else {
														sbBackup.append(sum2-diff-1);
													}
												}
												pathFound = true;
												break;
											}
										}
									}
									if(diff != 0) {
										i += skip1+skip2;
									}
									else if (diff==0 && a==sum1+1){
										for(int k=0; k<=sum1; k++) {
											tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(k));
											tempFromX = tempArray[0];
											tempFromY = tempArray[1];
											tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
											if(explored_map.getTile(tempFromX, tempFromY).getIsWaypoint())
												waypointCleared = true;
										}
										fromX = tempFromX;
										fromY = tempFromY;
										robotHeading = tempFromH;
										if(waypointCleared) {
											sbConfirmed.append(sb.substring(i, i+skip1));
										}
										else {
											sbBackup.append(sb.substring(i, i+skip1));
										}
										i+=skip1-1;
									}
								}
							}
							else {
//								sbBackup.append(sb.substring(i,i+skip1+skip2+1));
//								i += skip1+skip2;
								tempX = fromX;
								tempY = fromY;
								tempH = robotHeading;
								tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, String.valueOf(sum1));
								tempX = tempArray[0];
								tempY = tempArray[1];
								tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
								tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, "R");
								tempX = tempArray[0];
								tempY = tempArray[1];
								tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
								pathFound = false;
								for(a=0; a<=sum1; a++) {
									if(pathFound) {
										break;
									}
									tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(a));
									tempFromX = tempArray[0];
									tempFromY = tempArray[1];
									tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
									for(diff=sum2; diff>0; diff--) {
										tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, String.valueOf(diff));
										endX = tempArray[0];
										endY = tempArray[1];
										endH = RobotConstants.HEADING.fromInteger(tempArray[2]);
										if(this.checkDiagonalPathNoObstacle(tempFromX, tempFromY, endX, endY)) {
											if(!waypointCleared) {
												if(this.checkDiagonalPathWaypoint(tempFromX, tempFromY, endX, endY)) {
													sbConfirmed.append(sbBackup);
													waypointCleared = true;
												}
											}
											if(waypointCleared) {
												if(a==0) {
													System.out.println("HERE9");
													System.out.println("Don't append.");
												}
												else if(a>10) {
													for(int k=0; k<=a; k++) {
														tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(k));
														tempFromX = tempArray[0];
														tempFromY = tempArray[1];
														tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
														if(explored_map.getTile(tempFromX, tempFromY).getIsWaypoint())
															waypointCleared = true;
													}
													sbConfirmed.append(9);
													sbConfirmed.append((a-11) == 0 ? 0 : (a-11));
												}
												else {
													for(int k=0; k<=a; k++) {
														tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(k));
														tempFromX = tempArray[0];
														tempFromY = tempArray[1];
														tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
														if(explored_map.getTile(tempFromX, tempFromY).getIsWaypoint())
															waypointCleared = true;
													}
													sbConfirmed.append(a-1);
												}
												sbConfirmed.append(this.diagonalFastestTurningStr(tempFromX, tempFromY, tempFromH, endX, endY, endH));
											}
											else {
												if(a==0) {
													System.out.println("HERE10");
													System.out.println("Don't append.");
												}
												else if(a>10) {
													sbBackup.append(9);
													sbBackup.append((a-11) == 0 ? 0 : (a-11));
												}
												else {
													sbBackup.append(a-1);
												}
												sbBackup.append(this.diagonalFastestTurningStr(tempFromX, tempFromY, tempFromH, endX, endY, endH));
											}
											fromX = endX;
											fromY = endY;
											robotHeading = endH;
											if(diff != sum2) {
												if(sum2-diff>10) {
													sbBackup.append(9);
													sbBackup.append((sum2-diff-11) == 0 ? 0 : (sum2-diff-11));
												}
												else {
													sbBackup.append(sum2-diff-1);
												}
											}
											pathFound = true;
											break;
										}
									}
								}
								if(diff != 0) {
									i += skip1+skip2;
								}
								else if (diff==0 && a==sum1+1){
									for(int k=0; k<=sum1; k++) {
										tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(k));
										tempFromX = tempArray[0];
										tempFromY = tempArray[1];
										tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
										if(explored_map.getTile(tempFromX, tempFromY).getIsWaypoint())
											waypointCleared = true;
									}
									fromX = tempFromX;
									fromY = tempFromY;
									robotHeading = tempFromH;
									if(waypointCleared) {
										sbConfirmed.append(sb.substring(i, i+skip1));
									}
									else {
										sbBackup.append(sb.substring(i, i+skip1));
									}
									i+=skip1-1;
								}
							}
						}
					}
					else { //Send string to backup or confirmed?
						sbBackup.append(sb.substring(i,i+skip1));
						i += skip1-1;
					}
				}
				else { //Send string to backup or confirmed?
					sbBackup.append(sb.substring(i,i+skip1));
					i += skip1-1;
				}
			}
			else if (sb.substring(i, i+1).compareToIgnoreCase("U") == 0) { // R
				skip1 = this.numCharToSkipIfFront(sb.substring(i+1));
				sum1 = this.goFrontDist(sb.substring(i+1, i+skip1+1));
				if(i+skip1+2<=sb.length()) {
					if(sb.substring(i+skip1+1, i+skip1+2).compareToIgnoreCase("L") == 0) { // L
						skip2 = this.numCharToSkipIfFront(sb.substring(i+skip1+2));
						sum2 = this.goFrontDist(sb.substring(i+skip1+2, i+skip1+skip2+2));
						if(i+skip1+skip2+3<=sb.length()) {
							if(sb.substring(i+skip1+skip2+2, i+skip1+skip2+3).compareToIgnoreCase("U") == 0) { // R
								skip3 = this.numCharToSkipIfFront(sb.substring(i+skip1+skip2+3));
								sum3 = this.goFrontDist(sb.substring(i+skip1+skip2+3, i+skip1+skip2+skip3+3));
								if(i+skip1+skip2+skip3+4<=sb.length()) {
									if(sb.substring(i+skip1+skip2+skip3+3, i+skip1+skip2+skip3+4).compareToIgnoreCase("L") == 0) { // L
										skip4 = this.numCharToSkipIfFront(sb.substring(i+skip1+skip2+skip3+4));
										sum4 = this.goFrontDist(sb.substring(i+skip1+skip2+skip3+4, i+skip1+skip2+skip3+skip4+4));
										
										tempX = fromX;
										tempY = fromY;
										tempH = robotHeading;
										tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, "R");
										tempX = tempArray[0];
										tempY = tempArray[1];
										tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
										
										fromX = tempX;
										fromY = tempY;
										robotHeading = tempH;
										
										tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, String.valueOf(sum1));
										tempX = tempArray[0];
										tempY = tempArray[1];
										tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
										tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, "L");
										tempX = tempArray[0];
										tempY = tempArray[1];
										tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
										tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, String.valueOf(sum2));
										tempX = tempArray[0];
										tempY = tempArray[1];
										tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
										tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, "R");
										tempX = tempArray[0];
										tempY = tempArray[1];
										tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
										tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, String.valueOf(sum3));
										tempX = tempArray[0];
										tempY = tempArray[1];
										tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
										
										pathFound = false;
										for(a=0; a<=sum1; a++) {
											if(pathFound) {
												break;
											}
											tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(a));
											tempFromX = tempArray[0];
											tempFromY = tempArray[1];
											tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
											for(diff=sum4; diff>0; diff--) {
												tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, String.valueOf(diff));
												endX = tempArray[0];
												endY = tempArray[1];
												endH = RobotConstants.HEADING.fromInteger(tempArray[2]);
												if(this.checkDiagonalPathNoObstacle(tempFromX, tempFromY, endX, endY)) {
													if(!waypointCleared) {
														if(this.checkDiagonalPathWaypoint(tempFromX, tempFromY, endX, endY)) {
															sbConfirmed.append(sbBackup);
															waypointCleared = true;
														}
													}
													if(waypointCleared) {
														if(a==0) {
															System.out.println("HERE1");
															System.out.println("Don't append.");
														}
														else if(a>10) {
															for(int k=0; k<=a; k++) {
																tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(k));
																tempFromX = tempArray[0];
																tempFromY = tempArray[1];
																tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
																if(explored_map.getTile(tempFromX, tempFromY).getIsWaypoint())
																	waypointCleared = true;
															}
															sbConfirmed.append(9);
															sbConfirmed.append((a-11) == 0 ? 0 : (a-11));
														}
														else {
															for(int k=0; k<=a; k++) {
																tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(k));
																tempFromX = tempArray[0];
																tempFromY = tempArray[1];
																tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
																if(explored_map.getTile(tempFromX, tempFromY).getIsWaypoint())
																	waypointCleared = true;
															}
															sbConfirmed.append(a-1);
														}
														sbConfirmed.append(this.diagonalFastestTurningStr(tempFromX, tempFromY, tempFromH, endX, endY, endH));
													}
													else {
														if(a==0) {
															System.out.println("HERE2");
															System.out.println("Don't append.");
														}
														else if(a>10) {
															sbBackup.append(9);
															sbBackup.append((a-11) == 0 ? 0 : (a-11));
														}
														else {
															sbBackup.append(a-1);
														}
														sbBackup.append(this.diagonalFastestTurningStr(tempFromX, tempFromY, tempFromH, endX, endY, endH));
													}
													fromX = endX;
													fromY = endY;
													robotHeading = endH;
													if(diff != sum4) {
														if(sum4-diff>10) {
															sbBackup.append(9);
															sbBackup.append((sum4-diff-11) == 0 ? 0 : (sum4-diff-11));
														}
														else {
															sbBackup.append(sum4-diff-1);
														}
													}
													pathFound = true;
													break;
												}
											}
										}
										if(diff != 0) {
											i += skip1+skip2+skip3+skip4+3;
										}
										else if (diff==0 && a==sum1+1){
											for(int k=0; k<=sum1; k++) {
												tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(k));
												tempFromX = tempArray[0];
												tempFromY = tempArray[1];
												tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
												if(explored_map.getTile(tempFromX, tempFromY).getIsWaypoint())
													waypointCleared = true;
											}
											fromX = tempFromX;
											fromY = tempFromY;
											robotHeading = tempFromH;
											if(waypointCleared) {
												sbConfirmed.append(sb.substring(i, i+skip1+1));
											}
											else {
												sbBackup.append(sb.substring(i, i+skip1+1));
											}
											i+=skip1;
										}
									}
									else {
										tempX = fromX;
										tempY = fromY;
										tempH = robotHeading;
										tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, "R");
										tempX = tempArray[0];
										tempY = tempArray[1];
										tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
										
										fromX = tempX;
										fromY = tempY;
										robotHeading = tempH;
										
										tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, String.valueOf(sum1));
										tempX = tempArray[0];
										tempY = tempArray[1];
										tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
										tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, "L");
										tempX = tempArray[0];
										tempY = tempArray[1];
										tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
										tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, String.valueOf(sum2));
										tempX = tempArray[0];
										tempY = tempArray[1];
										tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
										tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, "R");
										tempX = tempArray[0];
										tempY = tempArray[1];
										tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
										pathFound = false;
										for(a=0; a<=sum1; a++) {
											if(pathFound) {
												break;
											}
											System.out.println("fromX: " + fromX);
											System.out.println("fromY: " + fromY);
											tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(a));
											tempFromX = tempArray[0];
											tempFromY = tempArray[1];
											tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
											System.out.println("sum3: " + sum3);
											for(diff=sum3; diff>0; diff--) {
												tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, String.valueOf(diff));
												endX = tempArray[0];
												endY = tempArray[1];
												endH = RobotConstants.HEADING.fromInteger(tempArray[2]);
												System.out.println("endX: " + endX);
												System.out.println("endY: " + endY);
												if(this.checkDiagonalPathNoObstacle(tempFromX, tempFromY, endX, endY)) {
													System.out.println("Diagonal Path No Obstacles!");
													if(!waypointCleared) {
														if(this.checkDiagonalPathWaypoint(tempFromX, tempFromY, endX, endY)) {
															sbConfirmed.append(sbBackup);
															waypointCleared = true;
														}
													}
													System.out.println("waypointCleared: " + waypointCleared);
													if(waypointCleared) {
														if(a==0) {
															System.out.println("HERE3");
															System.out.println("Don't append.");
														}
														else if(a>10) {
															for(int k=0; k<=a; k++) {
																tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(k));
																tempFromX = tempArray[0];
																tempFromY = tempArray[1];
																tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
																if(explored_map.getTile(tempFromX, tempFromY).getIsWaypoint())
																	waypointCleared = true;
															}
															sbConfirmed.append(9);
															sbConfirmed.append((a-11) == 0 ? 0 : (a-11));
														}
														else {
															for(int k=0; k<=a; k++) {
																tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(k));
																tempFromX = tempArray[0];
																tempFromY = tempArray[1];
																tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
																if(explored_map.getTile(tempFromX, tempFromY).getIsWaypoint())
																	waypointCleared = true;
															}
															sbConfirmed.append(a-1);
														}
														sbConfirmed.append(this.diagonalFastestTurningStr(tempFromX, tempFromY, tempFromH, endX, endY, endH));
													}
													else {
														if(a==0) {
															System.out.println("HERE4");
															System.out.println("Don't append.");
														}
														else if(a>10) {
															sbBackup.append(9);
															sbBackup.append((a-11) == 0 ? 0 : (a-11));
														}
														else {
															sbBackup.append(a-1);
														}
														sbBackup.append(this.diagonalFastestTurningStr(tempFromX, tempFromY, tempFromH, endX, endY, endH));
													}
													fromX = endX;
													fromY = endY;
													robotHeading = endH;
													if(diff != sum3) {
														if(sum3-diff>10) {
															sbBackup.append(9);
															sbBackup.append((sum3-diff-11) == 0 ? 0 : (sum3-diff-11));
														}
														else {
															sbBackup.append(sum3-diff-1);
														}
													}
													pathFound = true;
													break;
												}
											}
										}
										if(diff != 0) {
											i += skip1+skip2+skip3+2;
										}
										else if (diff==0 && a==sum1+1){
											for(int k=0; k<=sum1; k++) {
												tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(k));
												tempFromX = tempArray[0];
												tempFromY = tempArray[1];
												tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
												if(explored_map.getTile(tempFromX, tempFromY).getIsWaypoint())
													waypointCleared = true;
											}
											fromX = tempFromX;
											fromY = tempFromY;
											robotHeading = tempFromH;
											if(waypointCleared) {
												sbConfirmed.append(sb.substring(i, i+skip1+1));
											}
											else {
												sbBackup.append(sb.substring(i, i+skip1+1));
											}
											i+=skip1;
										}
									}
								}
								else {
									tempX = fromX;
									tempY = fromY;
									tempH = robotHeading;
									tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, "R");
									tempX = tempArray[0];
									tempY = tempArray[1];
									tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
									
									fromX = tempX;
									fromY = tempY;
									robotHeading = tempH;
									
									tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, String.valueOf(sum1));
									tempX = tempArray[0];
									tempY = tempArray[1];
									tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
									tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, "L");
									tempX = tempArray[0];
									tempY = tempArray[1];
									tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
									tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, String.valueOf(sum2));
									tempX = tempArray[0];
									tempY = tempArray[1];
									tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
									tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, "R");
									tempX = tempArray[0];
									tempY = tempArray[1];
									tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
									pathFound = false;
									for(a=0; a<=sum1; a++) {
										if(pathFound) {
											break;
										}
										tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(a));
										tempFromX = tempArray[0];
										tempFromY = tempArray[1];
										tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
										for(diff=sum3; diff>0; diff--) {
											tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, String.valueOf(diff));
											endX = tempArray[0];
											endY = tempArray[1];
											endH = RobotConstants.HEADING.fromInteger(tempArray[2]);
											if(this.checkDiagonalPathNoObstacle(tempFromX, tempFromY, endX, endY)) {
												if(!waypointCleared) {
													if(this.checkDiagonalPathWaypoint(tempFromX, tempFromY, endX, endY)) {
														sbConfirmed.append(sbBackup);
														waypointCleared = true;
													}
												}
												if(waypointCleared) {
													if(a==0) {
														System.out.println("HERE5");
														System.out.println("Don't append.");
													}
													else if(a>10) {
														for(int k=0; k<=a; k++) {
															tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(k));
															tempFromX = tempArray[0];
															tempFromY = tempArray[1];
															tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
															if(explored_map.getTile(tempFromX, tempFromY).getIsWaypoint())
																waypointCleared = true;
														}
														sbConfirmed.append(9);
														sbConfirmed.append((a-11) == 0 ? 0 : (a-11));
													}
													else {
														for(int k=0; k<=a; k++) {
															tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(k));
															tempFromX = tempArray[0];
															tempFromY = tempArray[1];
															tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
															if(explored_map.getTile(tempFromX, tempFromY).getIsWaypoint())
																waypointCleared = true;
														}
														sbConfirmed.append(a-1);
													}
													sbConfirmed.append(this.diagonalFastestTurningStr(tempFromX, tempFromY, tempFromH, endX, endY, endH));
												}
												else {
													if(a==0) {
														System.out.println("HERE6");
														System.out.println("Don't append.");
													}
													else if(a>10) {
														sbBackup.append(9);
														sbBackup.append((a-11) == 0 ? 0 : (a-11));
													}
													else {
														sbBackup.append(a-1);
													}
													sbBackup.append(this.diagonalFastestTurningStr(tempFromX, tempFromY, tempFromH, endX, endY, endH));
												}
												fromX = endX;
												fromY = endY;
												robotHeading = endH;
												if(diff != sum3) {
													if(sum3-diff>10) {
														sbBackup.append(9);
														sbBackup.append((sum3-diff-11) == 0 ? 0 : (sum3-diff-11));
													}
													else {
														sbBackup.append(sum3-diff-1);
													}
												}
												pathFound = true;
												break;
											}
										}
									}
									if(diff != 0) {
										i += skip1+skip2+skip3+2;
									}
									else if (diff==0 && a==sum1+1){
										for(int k=0; k<=sum1; k++) {
											tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(k));
											tempFromX = tempArray[0];
											tempFromY = tempArray[1];
											tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
											if(explored_map.getTile(tempFromX, tempFromY).getIsWaypoint())
												waypointCleared = true;
										}
										fromX = tempFromX;
										fromY = tempFromY;
										robotHeading = tempFromH;
										if(waypointCleared) {
											sbConfirmed.append(sb.substring(i, i+skip1+1));
										}
										else {
											sbBackup.append(sb.substring(i, i+skip1+1));
										}
										i+=skip1;
									}
								}
							}
							else {
								tempX = fromX;
								tempY = fromY;
								tempH = robotHeading;
								tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, "R");
								tempX = tempArray[0];
								tempY = tempArray[1];
								tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
								
								fromX = tempX;
								fromY = tempY;
								robotHeading = tempH;
								
								tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, String.valueOf(sum1));
								tempX = tempArray[0];
								tempY = tempArray[1];
								tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
								tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, "L");
								tempX = tempArray[0];
								tempY = tempArray[1];
								tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
								pathFound = false;
								for(a=0; a<=sum1; a++) {
									if(pathFound) {
										break;
									}
									tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(a));
									tempFromX = tempArray[0];
									tempFromY = tempArray[1];
									tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
									for(diff=sum2; diff>0; diff--) {
										tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, String.valueOf(diff));
										endX = tempArray[0];
										endY = tempArray[1];
										endH = RobotConstants.HEADING.fromInteger(tempArray[2]);
										if(this.checkDiagonalPathNoObstacle(tempFromX, tempFromY, endX, endY)) {
											if(!waypointCleared) {
												if(this.checkDiagonalPathWaypoint(tempFromX, tempFromY, endX, endY)) {
													sbConfirmed.append(sbBackup);
													waypointCleared = true;
												}
											}
											if(waypointCleared) {
												if(a==0) {
													System.out.println("HERE7");
													System.out.println("Don't append.");
												}
												else if(a>10) {
													for(int k=0; k<=a; k++) {
														tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(k));
														tempFromX = tempArray[0];
														tempFromY = tempArray[1];
														tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
														if(explored_map.getTile(tempFromX, tempFromY).getIsWaypoint())
															waypointCleared = true;
													}
													sbConfirmed.append(9);
													sbConfirmed.append((a-11) == 0 ? 0 : (a-11));
												}
												else {
													for(int k=0; k<=a; k++) {
														tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(k));
														tempFromX = tempArray[0];
														tempFromY = tempArray[1];
														tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
														if(explored_map.getTile(tempFromX, tempFromY).getIsWaypoint())
															waypointCleared = true;
													}
													sbConfirmed.append(a-1);
												}
												sbConfirmed.append(this.diagonalFastestTurningStr(tempFromX, tempFromY, tempFromH, endX, endY, endH));
											}
											else {
												if(a==0) {
													System.out.println("HERE8");
													System.out.println("Don't append.");
												}
												else if(a>10) {
													sbBackup.append(9);
													sbBackup.append((a-11) == 0 ? 0 : (a-11));
												}
												else {
													sbBackup.append(a-1);
												}
												sbBackup.append(this.diagonalFastestTurningStr(tempFromX, tempFromY, tempFromH, endX, endY, endH));
											}
											fromX = endX;
											fromY = endY;
											robotHeading = endH;
											if(diff != sum2) {
												if(sum2-diff>10) {
													sbBackup.append(9);
													sbBackup.append((sum2-diff-11) == 0 ? 0 : (sum2-diff-11));
												}
												else {
													sbBackup.append(sum2-diff-1);
												}
											}
											pathFound = true;
											break;
										}
									}
								}
								if(diff != 0) {
									i += skip1+skip2+1;
								}
								else if (diff==0 && a==sum1+1){
									for(int k=0; k<=sum1; k++) {
										tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(k));
										tempFromX = tempArray[0];
										tempFromY = tempArray[1];
										tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
										if(explored_map.getTile(tempFromX, tempFromY).getIsWaypoint())
											waypointCleared = true;
									}
									fromX = tempFromX;
									fromY = tempFromY;
									robotHeading = tempFromH;
									if(waypointCleared) {
										sbConfirmed.append(sb.substring(i, i+skip1+1));
									}
									else {
										sbBackup.append(sb.substring(i, i+skip1+1));
									}
									i+=skip1;
								}
							}
						}
						else {
							tempX = fromX;
							tempY = fromY;
							tempH = robotHeading;
							tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, "R");
							tempX = tempArray[0];
							tempY = tempArray[1];
							tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
							
							fromX = tempX;
							fromY = tempY;
							robotHeading = tempH;
							
							tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, String.valueOf(sum1));
							tempX = tempArray[0];
							tempY = tempArray[1];
							tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
							tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, "L");
							tempX = tempArray[0];
							tempY = tempArray[1];
							tempH = RobotConstants.HEADING.fromInteger(tempArray[2]);
							pathFound = false;
							for(a=0; a<=sum1; a++) {
								if(pathFound) {
									break;
								}
								tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(a));
								tempFromX = tempArray[0];
								tempFromY = tempArray[1];
								tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
								for(diff=sum2; diff>0; diff--) {
									tempArray = this.newCoordinatesAndHeadingByDistance(tempX, tempY, tempH, String.valueOf(diff));
									endX = tempArray[0];
									endY = tempArray[1];
									endH = RobotConstants.HEADING.fromInteger(tempArray[2]);
									if(this.checkDiagonalPathNoObstacle(tempFromX, tempFromY, endX, endY)) {
										if(!waypointCleared) {
											if(this.checkDiagonalPathWaypoint(tempFromX, tempFromY, endX, endY)) {
												sbConfirmed.append(sbBackup);
												waypointCleared = true;
											}
										}
										if(waypointCleared) {
											if(a==0) {
												System.out.println("HERE9");
												System.out.println("Don't append.");
											}
											else if(a>10) {
												for(int k=0; k<=a; k++) {
													tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(k));
													tempFromX = tempArray[0];
													tempFromY = tempArray[1];
													tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
													if(explored_map.getTile(tempFromX, tempFromY).getIsWaypoint())
														waypointCleared = true;
												}
												sbConfirmed.append(9);
												sbConfirmed.append((a-11) == 0 ? 0 : (a-11));
											}
											else {
												for(int k=0; k<=a; k++) {
													tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(k));
													tempFromX = tempArray[0];
													tempFromY = tempArray[1];
													tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
													if(explored_map.getTile(tempFromX, tempFromY).getIsWaypoint())
														waypointCleared = true;
												}
												sbConfirmed.append(a-1);
											}
											sbConfirmed.append(this.diagonalFastestTurningStr(tempFromX, tempFromY, tempFromH, endX, endY, endH));
										}
										else {
											if(a==0) {
												System.out.println("HERE10");
												System.out.println("Don't append.");
											}
											else if(a>10) {
												sbBackup.append(9);
												sbBackup.append((a-11) == 0 ? 0 : (a-11));
											}
											else {
												sbBackup.append(a-1);
											}
											sbBackup.append(this.diagonalFastestTurningStr(tempFromX, tempFromY, tempFromH, endX, endY, endH));
										}
										fromX = endX;
										fromY = endY;
										robotHeading = endH;
										if(diff != sum2) {
											if(sum2-diff>10) {
												sbBackup.append(9);
												sbBackup.append((sum2-diff-11) == 0 ? 0 : (sum2-diff-11));
											}
											else {
												sbBackup.append(sum2-diff-1);
											}
										}
										pathFound = true;
										break;
									}
								}
							}
							if(diff != 0) {
								i += skip1+skip2+1;
							}
							else if (diff==0 && a==sum1+1){
								for(int k=0; k<=sum1; k++) {
									tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, String.valueOf(k));
									tempFromX = tempArray[0];
									tempFromY = tempArray[1];
									tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
									if(explored_map.getTile(tempFromX, tempFromY).getIsWaypoint())
										waypointCleared = true;
								}
								fromX = tempFromX;
								fromY = tempFromY;
								robotHeading = tempFromH;
								if(waypointCleared) {
									sbConfirmed.append(sb.substring(i, i+skip1+1));
								}
								else {
									sbBackup.append(sb.substring(i, i+skip1+1));
								}
								i+=skip1;
							}
						}
					}
					else {
						sbBackup.append(sb.substring(i,i+skip1+1));
						i += skip1;
					}
				}
				else {
					sbBackup.append(sb.substring(i,i+skip1+1));
					i += skip1;
				}
			}
			else { // 'L'
				tempArray = this.newCoordinatesAndHeadingByDistance(fromX, fromY, robotHeading, "L");
				tempFromX = tempArray[0];
				tempFromY = tempArray[1];
				tempFromH = RobotConstants.HEADING.fromInteger(tempArray[2]);
				fromX = tempFromX;
				fromY = tempFromY;
				robotHeading = tempFromH;
				if(waypointCleared) {
					sbConfirmed.append(sb.substring(i, i+1));
				}
				else {
					sbBackup.append(sb.substring(i, i+1));
				}
			}
		}
		System.out.println("backup: " + sbBackup.toString());
		System.out.println("confirmed: " + sbConfirmed.toString());
		return sbConfirmed.toString();
	}
	
	public String diagonalFastestTurningStr(int fromX, int fromY, RobotConstants.HEADING fromH, int toX, int toY, RobotConstants.HEADING toH) {
		StringBuilder sb = new StringBuilder();
		double diffY = fromY - toY;
		double diffX = fromX - toX; 
		double degs = Math.toDegrees(Math.atan(Math.abs(diffX)/Math.abs(diffY))); //ADD CASE
		int nearestDeg = (int) Math.round(degs);
		int counterDeg = (int) Math.round(90 - nearestDeg);
		String nearestDegStr = nearestDeg < 10 ? "0" + String.valueOf(nearestDeg) : String.valueOf(nearestDeg);
		String counterDegStr = counterDeg < 10 ? "0" + String.valueOf(counterDeg) : String.valueOf(counterDeg);
		System.out.println("Hyp: " + Math.sqrt((diffX*diffX) + (diffY*diffY)));
		int dist = (int) Math.round(Math.sqrt((diffX*diffX) + (diffY*diffY)));
		switch(fromH){
		case NORTH:
			if(toX>fromX) {
				sb.append("Y");
			}
			else {
				sb.append("X");
			}
			sb.append(nearestDegStr);
			if(dist>20) {
				sb.append(9);
				sb.append(9);
				sb.append((dist-21) == 0 ? 0 : (dist-21));
			}
			else if(dist>10) {
				sb.append(9);
				sb.append((dist-11) == 0 ? 0 : (dist-11));
			}
			else {
				sb.append(dist-1);
			}
			switch(toH) {
			case NORTH:
				if(toX>fromX) {
					sb.append("X");
				}
				else {
					sb.append("Y");
				}
				sb.append(nearestDegStr);
				break;
			case EAST:
				if(toX>fromX) {
					sb.append("Y");
					sb.append(counterDegStr);
				}
				else {
					sb.append("Y");
					sb.append(nearestDegStr);
					sb.append("U");
				}
				break;
			case WEST:
				if(toX>fromX) {
					sb.append("X");
					sb.append(nearestDegStr);
					sb.append("L");
				}
				else {
					sb.append("X");
					sb.append(counterDegStr);
				}
				break;
			case SOUTH:
				if(toX>fromX) {
					sb.append("Y");
					sb.append(counterDegStr);
					sb.append("U");
				}
				else {
					sb.append("X");
					sb.append(counterDegStr);
					sb.append("L");
				}
				break;
				default:break;
				}
			return sb.toString();
		case EAST:
			if(toX>fromX) { 
				if(toY<fromY) {
					sb.append("X");
				}
				else {
					sb.append("Y");
				}
			}
			///////////////////////// points at top-left and bottom-left
			sb.append(nearestDegStr);
			if(dist>20) {
				sb.append(9);
				sb.append(9);
				sb.append((dist-21) == 0 ? 0 : (dist-21));
			}
			else if(dist>10) {
				sb.append(9);
				sb.append((dist-11) == 0 ? 0 : (dist-11));
			}
			else {
				sb.append(dist-1);
			}
			switch(toH) {
			case NORTH:
				if(toX>fromX) {
					if(toY<fromY) {
						sb.append("X");
						sb.append(counterDegStr);
					}
					else {
						sb.append("X");
						sb.append(nearestDegStr);
						sb.append("L");
					}
				}
				break;
			case EAST:
				if(toX>fromX) {
					if(toY<fromY) {
						sb.append("Y");
						sb.append(nearestDegStr);
					}
					else {
						sb.append("X");
						sb.append(nearestDegStr);
					}
				}
				break;
			case WEST:
				if(toX>fromX) {
					if(toY<fromY) {
						sb.append("X");
						sb.append(counterDegStr);
						sb.append("L");
					}
					else {
						sb.append("Y");
						sb.append(counterDegStr);
						sb.append("U");
					}
				}
				break;
			case SOUTH:
				if(toX>fromX) {
					if(toY<fromY) {
						sb.append("Y");
						sb.append(nearestDegStr);
						sb.append("U");
					}
					else {
						sb.append("Y");
						sb.append(counterDegStr);
					}
				}
				break;
				default:break;
			}
			return sb.toString();
		case WEST:
			if(toX<fromX) {
				if(toY<fromY) {
					sb.append("Y");
				}
				else {
					sb.append("X");
				}
			}
			sb.append(nearestDegStr);
			if(dist>20) {
				sb.append(9);
				sb.append(9);
				sb.append((dist-21) == 0 ? 0 : (dist-21));
			}
			else if(dist>10) {
				sb.append(9);
				sb.append((dist-11) == 0 ? 0 : (dist-11));
			}
			else {
				sb.append(dist-1);
			}
			switch(toH) {
			case NORTH:
				if(toX<fromX) {
					if(toY<fromY) {
						sb.append("Y");
						sb.append(counterDegStr);
					}
					else {
						sb.append("Y");
						sb.append(nearestDegStr);
						sb.append("U");
					}
				}
				break;
			case WEST:
				if(toX<fromX) {
					if(toY<fromY) {
						sb.append("X");
						sb.append(nearestDegStr);
					}
					else {
						sb.append("Y");
						sb.append(nearestDegStr);
					}
				}
				break;
			case EAST:
				if(toX<fromX) {
					if(toY<fromY) {
						sb.append("Y");
						sb.append(counterDegStr);
						sb.append("U");
					}
					else {
						sb.append("X");
						sb.append(counterDegStr);
						sb.append("L");
					}
				}
				break;
			case SOUTH:
				if(toX<fromX) {
					if(toY<fromY) {
						sb.append("X");
						sb.append(nearestDegStr);
						sb.append("L");
					}
					else {
						sb.append("X");
						sb.append(counterDegStr);
					}
				}
				break;
				default: break;
			}
			return sb.toString();
			//////////// fromH south case needed?
			default: break;
		}
		return sb.toString();
	}

	// true if diagonal path is valid (no obstacles)
	public boolean checkDiagonalPathNoObstacle(int fromX, int fromY, int toX, int toY) {
		if (fromY == toY||fromX == toX)
			return false;
		double diffY = fromY - toY;
		double diffX = fromX - toX;
		double degs = Math.toDegrees(Math.atan((diffX)/(diffY)));
		if(toX>fromX && fromY>toY) {
			degs += 360;
		}
		else if(toX>fromX && toY>fromY) {
			degs += 180;
		}
		else if(toX<fromX && toY>fromY) {
			degs += 180;
		}
		if(((int)degs) % 360 < 45) {
			for(int i=toY; i<=fromY; i++) {
				double ans = (i-toY)/(diffY/diffX) + toX;
				int rounded = (int) Math.round(ans);
				if (!explored_map.checkTileSurroundingLenient(rounded, i))
					return false;
			}
			return true;
		}
		else if(((int)degs) % 360 < 90) {
			for(int i=toX; i<=fromX; i++) {
				double ans = (i-toX)*(diffY/diffX) + toY;
				int rounded = (int) Math.round(ans);
				if (!explored_map.checkTileSurroundingLenient(i, rounded))
					return false;
			}
			return true;
		}
		else if(((int)degs) % 360 < 135) {
			for(int i=toX; i<=fromX; i++) {
				double ans = (i-toX)*(diffY/diffX) + toY;
				int rounded = (int) Math.round(ans);
				if (!explored_map.checkTileSurroundingLenient(i, rounded))
					return false;
			}
			return true;
		}
		else if(((int)degs) % 360 < 180) {
			for(int i=fromY; i<=toY; i++) {
				double ans = (i-toY)/(diffY/diffX) + toX;
				int rounded = (int) Math.round(ans);
				if (!explored_map.checkTileSurroundingLenient(rounded, i))
					return false;
			}
			return true;
		}
		else if(((int)degs) % 360 < 225) {
			for(int i=fromY; i<=toY; i++) {
				double ans = (i-toY)/(diffY/diffX) + toX;
				int rounded = (int) Math.round(ans);
				if (!explored_map.checkTileSurroundingLenient(rounded, i))
					return false;
			}
			return true;
		}
		else if(((int)degs) % 360 < 270) {
			for(int i=fromX; i<=toX; i++) {
				double ans = (i-toX)*(diffY/diffX) + toY;
				int rounded = (int) Math.round(ans);
				if (!explored_map.checkTileSurroundingLenient(i, rounded))
					return false;
			}
			return true;
		}
		else if(((int)degs) % 360 < 315) {
			for(int i=fromX; i<=toX; i++) {
				double ans = (i-toX)*(diffY/diffX) + toY;
				int rounded = (int) Math.round(ans);
				if (!explored_map.checkTileSurroundingLenient(i, rounded))
					return false;
			}
			return true;
		}
		else {
			for(int i=toY; i<=fromY; i++) {
				double ans = (i-toY)/(diffY/diffX) + toX;
				int rounded = (int) Math.round(ans);
				if (!explored_map.checkTileSurroundingLenient(rounded, i))
					return false;
			}
			return true;
		}
	}
	
	// true if diagonal path passes through waypoint
	public boolean checkDiagonalPathWaypoint(int fromX, int fromY, int toX, int toY) {
		if (fromY == toY||fromX == toX)
			return false;
		double diffY = fromY - toY;
		double diffX = fromX - toX;
		double degs = Math.toDegrees(Math.atan((diffX)/(diffY)));
		if(toX>fromX && fromY>toY) {
			degs += 360;
		}
		else if(toX>fromX && toY>fromY) {
			degs += 180;
		}
		else if(toX<fromX && toY>fromY) {
			degs += 180;
		}
		if(((int)degs) % 360 < 45) {
			for(int i=toY; i<=fromY; i++) {
				double ans = (i-toY)/(diffY/diffX) + toX;
				int rounded = (int) Math.round(ans);
				if (explored_map.checkWaypointTouched(rounded, i))
					return true;
			}
			return false;
		}
		else if(((int)degs) % 360 < 90) {
			for(int i=toX; i<=fromX; i++) {
				double ans = (i-toX)*(diffY/diffX) + toY;
				int rounded = (int) Math.round(ans);
				if (explored_map.checkWaypointTouched(i, rounded))
					return true;
			}
			return false;
		}
		else if(((int)degs) % 360 < 135) {
			for(int i=toX; i<=fromX; i++) {
				double ans = (i-toX)*(diffY/diffX) + toY;
				int rounded = (int) Math.round(ans);
				if (explored_map.checkWaypointTouched(i, rounded))
					return true;
			}
			return false;
		}
		else if(((int)degs) % 360 < 180) {
			for(int i=fromY; i<=toY; i++) {
				double ans = (i-toY)/(diffY/diffX) + toX;
				int rounded = (int) Math.round(ans);
				if (explored_map.checkWaypointTouched(rounded, i))
					return true;
			}
			return false;
		}
		else if(((int)degs) % 360 < 225) {
			for(int i=fromY; i<=toY; i++) {
				double ans = (i-toY)/(diffY/diffX) + toX;
				int rounded = (int) Math.round(ans);
				if (explored_map.checkWaypointTouched(rounded, i))
					return true;
			}
			return false;
		}
		else if(((int)degs) % 360 < 270) {
			for(int i=fromX; i<=toX; i++) {
				double ans = (i-toX)*(diffY/diffX) + toY;
				int rounded = (int) Math.round(ans);
				if (explored_map.checkWaypointTouched(i, rounded))
					return true;
			}
			return false;
		}
		else if(((int)degs) % 360 < 315) {
			for(int i=fromX; i<=toX; i++) {
				double ans = (i-toX)*(diffY/diffX) + toY;
				int rounded = (int) Math.round(ans);
				if (explored_map.checkWaypointTouched(i, rounded))
					return true;
			}
			return false;
		}
		else {
			for(int i=toY; i<=fromY; i++) {
				double ans = (i-toY)/(diffY/diffX) + toX;
				int rounded = (int) Math.round(ans);
				if (explored_map.checkWaypointTouched(rounded, i))
					return true;
			}
			return false;
		}
	}
	
	public int numCharToSkipIfFront(String str) {
		if(str.length()>1) {
			if(this.checkIfGoFront(str.substring(0,1)) && this.checkIfGoFront(str.substring(1,2))) {
				return 2;
			}
		}
		if(str.substring(0,1).compareToIgnoreCase("L") == 0 || str.substring(0,1).compareToIgnoreCase("X") == 0 || str.substring(0,1).compareToIgnoreCase("Y") == 0) {
			return 0;
		}
		else if(str.substring(0,1).compareToIgnoreCase("U") == 0 || str.substring(0,1).compareToIgnoreCase("R") == 0) {
			return 0;
		}
		else if(this.checkIfGoFront(str.substring(0,1)))
			return 1;
		else
			return -1;
	}
	
	public int goFrontDist(String str) {
		if(str.length() == 2) {
			if(isNumeric(str))
				return Integer.parseInt(str.substring(0,1)) + Integer.parseInt(str.substring(1,2)) + 2;
		}
		else if(str.length() == 3) {
			if(isNumeric(str))
				return Integer.parseInt(str.substring(0,1)) + Integer.parseInt(str.substring(1,2)) + Integer.parseInt(str.substring(2,3)) + 3;
		}
		else if(str.length() == 1) {
			if(isNumeric(str))
				return Integer.parseInt(str) + 1;
		}
		return 0;
	}
	
	public boolean checkIfGoFront(String str) {
		if(isNumeric(str))
			return true;
		else
			return false;
	}
	
	public boolean isNumeric(String str) {
		  return str.matches("-?\\d+(\\.\\d+)?");  //match a number with optional '-' and decimal.
		}
	
	// command number would be distance
	public int[] newCoordinatesAndHeadingByDistance(int x, int y, RobotConstants.HEADING h, String command) {
		int[] result = new int[3];
		if(this.isNumeric(command)) {
			switch(h) {
			case NORTH:
				result[0] = x;
				result[1] = y - Integer.parseInt(command);
				result[2] = 0;
				break;
			case WEST:
				result[0] = x - Integer.parseInt(command);
				result[1] = y;
				result[2] = 1;
				break;
			case SOUTH:
				result[0] = x;
				result[1] = y + Integer.parseInt(command);
				result[2] = 2;
				break;
			case EAST:
				result[0] = x + Integer.parseInt(command);
				result[1] = y;
				result[2] = 3;
				break;
			default:
				break;
			}
		}
		else if (command.compareToIgnoreCase("U") == 0 || command.compareToIgnoreCase("R") == 0){
			switch(h) {
			case NORTH:
				result[0] = x;
				result[1] = y;
				result[2] = 3;
				break;
			case WEST:
				result[0] = x;
				result[1] = y;
				result[2] = 0;
				break;
			case SOUTH:
				result[0] = x;
				result[1] = y;
				result[2] = 1;
				break;
			case EAST:
				result[0] = x;
				result[1] = y;
				result[2] = 2;
				break;
			default:
				break;
			}
		}
		else {
			switch(h) {
			case NORTH:
				result[0] = x;
				result[1] = y;
				result[2] = 1;
				break;
			case WEST:
				result[0] = x;
				result[1] = y;
				result[2] = 2;
				break;
			case SOUTH:
				result[0] = x;
				result[1] = y;
				result[2] = 3;
				break;
			case EAST:
				result[0] = x;
				result[1] = y;
				result[2] = 0;
				break;
			default:
				break;
			}
		}
	return result;
	}
	
	// reading the movement string char by char
	public int[] newCoordinatesHeadingFromString(int x, int y, RobotConstants.HEADING h, String command) {
		int[] result = new int[3];
		if(this.isNumeric(command)) {
			switch(h) {
			case NORTH:
				result[0] = x;
				result[1] = y - Integer.parseInt(command) -1;
				result[2] = 0;
				break;
			case WEST:
				result[0] = x - Integer.parseInt(command) -1;
				result[1] = y;
				result[2] = 1;
				break;
			case SOUTH:
				result[0] = x;
				result[1] = y + Integer.parseInt(command) +1;
				result[2] = 2;
				break;
			case EAST:
				result[0] = x + Integer.parseInt(command) +1;
				result[1] = y;
				result[2] = 3;
				break;
			default:
				break;
			}
		}
		else if (command.compareToIgnoreCase("U") == 0 || command.compareToIgnoreCase("R") == 0){
			switch(h) {
			case NORTH:
				result[0] = x;
				result[1] = y;
				result[2] = 3;
				break;
			case WEST:
				result[0] = x;
				result[1] = y;
				result[2] = 0;
				break;
			case SOUTH:
				result[0] = x;
				result[1] = y;
				result[2] = 1;
				break;
			case EAST:
				result[0] = x;
				result[1] = y;
				result[2] = 2;
				break;
			default:
				break;
			}
		}
		else {
			switch(h) {
			case NORTH:
				result[0] = x;
				result[1] = y;
				result[2] = 1;
				break;
			case WEST:
				result[0] = x;
				result[1] = y;
				result[2] = 2;
				break;
			case SOUTH:
				result[0] = x;
				result[1] = y;
				result[2] = 3;
				break;
			case EAST:
				result[0] = x;
				result[1] = y;
				result[2] = 0;
				break;
			default:
				break;
			}
		}
	return result;
	}
	
}
