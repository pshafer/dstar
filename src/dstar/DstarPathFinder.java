/**
 * @author J. Levy, P. Shafer
 * Due 12/19/2014
 * 
 * DstarPathFinder is a path implements the D* algorithm in order to travers
 * a given map.
 * 
 * A large portion of this implementation is based off of the paper
 * 
 * "Optimal and Efficient Path Planning for Paritally-Known Environment"
 * by Anthony Stentz
 * 
 * 
 */
package dstar;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.PriorityQueue;

public class DstarPathFinder {

	// Define constant values for path costs
	public static final double NORMAL_COST = 1.0;
	public static final double ADJACENT_COST = 1.4;
	public static final double INFINITY_COST = 10000;
	
	// define cost table object
	private CostTable costs;
	// define map object
	private DstarMap map;
	// define open list data structure
	private PriorityQueue<DstarNode> openList;
	
	// provide DstarNode objects for local storage
	// of start, goal, and currentRobotPosition states
	private DstarNode start;
	private DstarNode goal;
	private DstarNode currentRobotPosition;
	
	/**
	 * Default constructor 
	 * 	
	 * @param map a valid DstarMap
	 */
	public DstarPathFinder(DstarMap map){
		
		costs = new CostTable();
		openList = new PriorityQueue<DstarNode>();
		this.map = map;
		
		start = map.getStart();
		currentRobotPosition = map.getRobotLocation();
		goal = map.getGoal();
		
		buildCostTable();
	}
	
	/**
	 * Call traverseMap to find the optimal path
	 */
	public void traverseMap(){
		
		// minimim_K will be used to store what is returned from processState()
		double minimum_K;
		
		// add the goal to the open list
		insert(goal, 0);

		// print the map
		
		map.print();
		
		// print the map as initialized and prompt user to start next seciont
		int userInput;
		try{
			System.out.println("Map has been initialized!");
			System.out.println("Ready to begin (press enter):");
			userInput = System.in.read();
		}catch(Exception e){
			
		}

		// loop process state until minimum_K == -1 or that start state is closed
		do{
			minimum_K = processState();					
		}while(minimum_K!=-1.0 && !start.getTag().equalsIgnoreCase("CLOSED"));
		
		// after processing initial loop of process state
		// print the current state of the map
		try{
			map.print();
			System.out.println("Map has been expanded from goal...");
			System.out.println("We will either begin tracing to goal, or the goal is unreachable (press enter):");
			userInput = System.in.read();
		}catch(Exception e){
			
		}
		
		// if mimimum_K == -1.0 then goal is unreachable
		if(minimum_K == -1.0){
			System.out.println("Goal is unreachable...");
			System.exit(0);
		}else{
			
			// start tracing path through back pointers
			// trace until the goal is reached or the next node is unknown
			do{
				DstarNode next;
				DstarNode here;
				boolean unknownFound = false;
				map.print();
				
				do{
					// start from where to robot is current located
					here = map.getRobotLocation();
					// get next node from current location back pointer
					next = here.getBackPointer();
					
					// is next state unknown?
					unknownFound = next.getState().equals("U");
					
					// next state non unknown?
					if(!unknownFound){
						
						// move the robot
						map.setRobotLocation(next);
						currentRobotPosition = map.getRobotLocation();
						
						// print the map, wait for user input
						map.print();
						try{
							System.out.println("Robot Moved!!");
							System.out.println("Move again (press enter):");
							userInput = System.in.read();
						}catch(Exception e){
							
						}
					}
				}while(currentRobotPosition != goal && !unknownFound);
				
				// if we reach the goal, exit
				if(currentRobotPosition == goal){
					System.out.println("Goal has been reached!");
					System.exit(0);
				}else{
					// we are not at the goal yet
					// let the user know we found an unknown obstacle
					// prompt for input to continue
					try{
						System.out.println("Unkown Obstacle!!!");
						System.out.println("Find another way (press enter):");
						userInput = System.in.read();
					}catch(Exception e){
						
					}
					
					// modify the cost from here to next to infinity
					modifyCost(here, next, INFINITY_COST);
					
					// loop process state again until
					// mimimum_K < h(next) or mimimum_K == -1.0
					do{
						minimum_K = processState();
					}while(minimum_K < next.getH() && minimum_K!=-1.0 &&  !currentRobotPosition.getTag().equalsIgnoreCase("CLOSED") );//*BUG FIX, we weren't stopping the process state when the current robot position was closed, this caused the bot to recalculate too much and overwrite the correct path backpointers*//&& !(currentRobotPosition.equals(openList.peek())));
										
					// if mimimum_K == -1.0 goal is unreachable, exit application
					if(minimum_K ==-1.0){
						System.out.println("Goal is unreachable!");
						System.exit(0);
					}
					
					// print the map, wait for user input
					map.print();
					try{
						System.out.println("Whew... I found my way!!");
						System.out.println("Move again (press enter):");
						userInput = System.in.read();
					}catch(Exception e){
						
					}
				}
				
			}while(true);
		}
	}
	
	/**
	 * Modifies the cost of the paths of the unknown neighbors to infinity
	 * 
	 * @param current
	 * @param neighbor
	 * @param newVal
	 * @return
	 */
	private double modifyCost(DstarNode current, DstarNode neighbor, double newVal){
		
		//costs.updateValue(current.getLabel(), neighbor.getLabel(), newVal);
				
		//discrepency.setH(INFINITY_COST);
		ArrayList<DstarNode> neighborList = map.getNeighbors(neighbor.getRow(), neighbor.getColumn());
		
		// update the arc path costs of neighbor to its neighbors
		for(DstarNode each: neighborList){
			costs.updateValue(neighbor.getLabel(), each.getLabel(), INFINITY_COST);
		}

		//neighbor.setH(newVal);
		
		// if neight is closed, then add it back to the open list with new h value
		if(neighbor.getTag().equalsIgnoreCase("CLOSED")){
			
			insert(neighbor,newVal);
		}
	
		// get the new minimum k value on the open list
		DstarNode current_Min = openList.peek();
		
		return current_Min.getK();
	}
	
	/**
	 * process the current node and sets neighbor values as deemed necessary
	 * @return
	 */
	private double processState(){
		double k_Old;
		ArrayList<DstarNode> neighbors;
		int userInput;
		// STEP 1
		// if open list is empty exit return -1
		if (openList.isEmpty())
		{
			return -1.0;
		}
		if (currentRobotPosition.getK() ==10000 && currentRobotPosition.getH() == 10000)
		{
			return -1.0;
			
		}
		
		// get node from head of openList (with mink)
		DstarNode currentNode = openList.remove();
		
		k_Old = currentNode.getK();//k_Old = Get Min K
		
		// and set it to closed	
		currentNode.setTag("CLOSED");//Delete X
		

		// STEP 2 Re-routing if necessary
		// refers to L4 - L7 in paper by Stentz
		if(k_Old < currentNode.getH()){
			
			System.out.println("Step 2 - Rerouting - node: " + currentNode.getLabel());;
	
			
			
			neighbors = map.getNeighbors(currentNode.getRow(), currentNode.getColumn());
			for(DstarNode neighbor : neighbors){
				System.out.println("Updating Neighbor: " + neighbor.getLabel());
				double costXthroughY = neighbor.getH() + costs.getValue(neighbor.getLabel(), currentNode.getLabel());
				
				if(neighbor.getH() <= k_Old &&
					currentNode.getH() > costXthroughY){
					System.out.println("-- step 2 -- set back pointer to " + neighbor.getLabel());
					System.out.println("current: " + currentNode.getLabel());
	
					currentNode.setBackPointer(neighbor);
					currentNode.setH(costXthroughY);
				}	
			}
		}
		
		// Step 3 - usually done in initial map state processing
		// refers to L8 - L13 in paper by Stentz
		if(k_Old == currentNode.getH()){
			//System.out.println("Start Step 3");
			
			neighbors = map.getNeighbors(currentNode.getRow(), currentNode.getColumn());
			
			for(DstarNode neighbor : neighbors){
				double costThroughX = currentNode.getH() + costs.getValue(currentNode.getLabel(), neighbor.getLabel());
				double roundedNumber = (double)Math.round(costThroughX * 10) / 10;
				if( neighbor.getTag().equalsIgnoreCase("NEW") ||
				    (neighbor.getBackPointer() == currentNode && neighbor.getH() != costThroughX ) ||
				    (neighbor.getBackPointer() != currentNode && neighbor.getH() > costThroughX )){
					
					neighbor.setBackPointer(currentNode);
					insert(neighbor, roundedNumber);
				}
			}
		}
		
		// Step 4
		// referces to L14 - L25 in paper by Stentz
		else{
			neighbors = map.getNeighbors(currentNode.getRow(), currentNode.getColumn());
			for(DstarNode neighbor : neighbors){
				double costThroughX = currentNode.getH() + costs.getValue(currentNode.getLabel(), neighbor.getLabel());
				double costXthroughY = neighbor.getH() + costs.getValue(neighbor.getLabel(), currentNode.getLabel());
				//double roundedCostThroughX = costThroughX;
			//	double roundedCostXThroughY = costXthroughY;
				
				double roundedCostThroughX = (double)Math.round(costThroughX * 10) / 10;
				double roundedCostXThroughY = (double)Math.round(costXthroughY * 10) / 10;
				double roundedH;
		
				// Step 5
				// refers to L16 - L18 paper by Stentz
				if(neighbor.getTag().equalsIgnoreCase("NEW") || (neighbor.getBackPointer() == currentNode && neighbor.getH() != costThroughX)){

					//routeFirstViaSecond(neighbor,currentNode);
					neighbor.setBackPointer(currentNode);
					insert(neighbor,roundedCostThroughX);
					

				}else if(neighbor.getBackPointer() != currentNode && neighbor.getH() > roundedCostThroughX){
					roundedH = (double)Math.round(currentNode.getH() * 10) / 10;
					insert(currentNode, roundedH);
					
				}else if(neighbor.getBackPointer() != currentNode && currentNode.getH() > roundedCostXThroughY && neighbor.getTag().equalsIgnoreCase("CLOSED") && neighbor.getH() > k_Old){
					roundedH = (double)Math.round(neighbor.getH() * 10) / 10;
					insert(neighbor, roundedH);
				}
			}
		}
		

		// set minK, of openList is not empty, then get the minK from open list
		double minK = -1.0;
		if(!openList.isEmpty()){
			minK = openList.peek().getK();
		}
		try{
			
			System.out.println("Planning....");
			System.out.println("Current Path:");
			map.print();
			System.out.println("Press ENTER to continue...");
			userInput = System.in.read();
		}catch(Exception e){
			
		}
		
		return minK;
		
	}
	
	/**
	 * routeFristViaSecond from Dr. Kays notes, however we don't use this
	 * 
	 * TODO remove
	 * 
	 * @param Y
	 * @param X
	 */
	private void routeFirstViaSecond(DstarNode Y, DstarNode X){
		Y.setBackPointer(X);
		Y.setH(X.getH() + costs.getValue(X.getLabel(),Y.getLabel()));
		if(Y.getTag().equalsIgnoreCase("NEW")){
			Y.setK(Y.getH());
		}
		
		/*
		// since we are using a priority queue
		// if Y is on the openlist and the K 
		if(Y.getTag().equals("OPEN")){
			openList.remove(Y);
		}
		*/
		Y.setTag("OPEN");
		openList.add(Y);
	}
	
	/**
	 * Update a nodes k and h value and inserts/reinserts a node into the open list
	 * 
	 * @param someNode
	 * @param newH
	 */
	private void insert(DstarNode someNode, double newH)
	{
		if (newH>10000)
		{
			newH = 10000;
		}
		double roundedH;
		double currentK;
		DstarNode temp;
		if (someNode.getTag().equalsIgnoreCase("NEW"))
		{
			someNode.setK(newH);
			someNode.setH(newH);
			someNode.setTag("OPEN");
			openList.add(someNode);
		}
		if (someNode.getTag().equalsIgnoreCase("OPEN"))
		{
			currentK=someNode.getK();
			
			openList.remove(someNode);
			
			someNode.setK(Math.min(currentK, newH));	
			openList.add(someNode);
			
		}
		if (someNode.getTag().equalsIgnoreCase("CLOSED"))
		{
			
			int userInput;
			roundedH = (double)Math.round(someNode.getH() * 10) / 10;
			someNode.setK(Math.min(roundedH, newH));
			someNode.setH(newH);
			someNode.setTag("OPEN");
			openList.add(someNode);
			
		}
	}
	
	/**
	 * Initialized the cost table
	 * 
	 * TODO a lot of this is repative, and should be fixed
	 * neightborOutofBounds should be a public method in DstarMap
	 */
	private void buildCostTable(){
		//loop all nodes
		// set cost between open nodes to 1 or 1.4
		
		int current_x=0;
		int current_y=0;
		
		DstarNode neighbor;
		for(int i=0; i < map.getRows(); i++){
			for(int j=0; j < map.getColumns(); j++){
				DstarNode current = map.getNode(i,j);
				
				// set top and bottom neighbor row values
				int top = i-1;
				int bottom = i+1;
				
				// set left and right neighbor column values
				int left = j-1;
				int right = j+1;
				
				// top left adjecent neighbor
				if(!neighborOutOfBounds(top,left)){
					
					neighbor = map.getNode(top, left);
					
					if(current.getState().equalsIgnoreCase("B") || neighbor.getState().equalsIgnoreCase("B")){
						costs.setValue(current.getLabel(), neighbor.getLabel(), INFINITY_COST);
					}else{
						costs.setValue(current.getLabel(), neighbor.getLabel(), ADJACENT_COST);
					}
					
				}
				
				// direct top neighbor
				if(!neighborOutOfBounds(top, j)){
					neighbor = map.getNode(top, j);
					
					if(current.getState().equalsIgnoreCase("B") || neighbor.getState().equalsIgnoreCase("B")){
						costs.setValue(current.getLabel(), neighbor.getLabel(), INFINITY_COST);
					}else{
						costs.setValue(current.getLabel(), neighbor.getLabel(), NORMAL_COST);
					}
				}
				
				// top right neighbor
				if(!neighborOutOfBounds(top,right)){
					neighbor = map.getNode(top, right);
					
					if(current.getState().equalsIgnoreCase("B") || neighbor.getState().equalsIgnoreCase("B")){
						costs.setValue(current.getLabel(), neighbor.getLabel(), INFINITY_COST);
					}else{
						costs.setValue(current.getLabel(), neighbor.getLabel(), ADJACENT_COST);
					}
				}
				
				// left direct neighbor
				if(!neighborOutOfBounds(i,left)){
					neighbor = map.getNode(i, left);
					
					if(current.getState().equalsIgnoreCase("B") || neighbor.getState().equalsIgnoreCase("B")){
						costs.setValue(current.getLabel(), neighbor.getLabel(), INFINITY_COST);
					}else{
						costs.setValue(current.getLabel(), neighbor.getLabel(), NORMAL_COST);
					}
				}
				
				// right direct neighbor
				if(!neighborOutOfBounds(i, right)){
					neighbor = map.getNode(i, right);
					
					if(current.getState().equalsIgnoreCase("B") || neighbor.getState().equalsIgnoreCase("B")){
						costs.setValue(current.getLabel(), neighbor.getLabel(), INFINITY_COST);
					}else{
						costs.setValue(current.getLabel(), neighbor.getLabel(), NORMAL_COST);
					}
				}
				
				// bottom left neighbor
				if(!neighborOutOfBounds(bottom,left)){
					neighbor = map.getNode(bottom, left);
					
					if(current.getState().equalsIgnoreCase("B") || neighbor.getState().equalsIgnoreCase("B")){
						costs.setValue(current.getLabel(), neighbor.getLabel(), INFINITY_COST);
					}else{
						costs.setValue(current.getLabel(), neighbor.getLabel(), ADJACENT_COST);
					}
				}
				
				// direct bottom neighbor
				if(!neighborOutOfBounds(bottom,j)){
					neighbor = map.getNode(bottom, j);
					
					if(current.getState().equalsIgnoreCase("B") || neighbor.getState().equalsIgnoreCase("B")){
						costs.setValue(current.getLabel(), neighbor.getLabel(), INFINITY_COST);
					}else{
						costs.setValue(current.getLabel(), neighbor.getLabel(), NORMAL_COST);
					}
				}
				
				// bottom right neighbor
				if(!neighborOutOfBounds(bottom,right)){
					neighbor = map.getNode(bottom, right);
					
					if(current.getState().equalsIgnoreCase("B") || neighbor.getState().equalsIgnoreCase("B")){
						costs.setValue(current.getLabel(), neighbor.getLabel(), INFINITY_COST);
					}else{
						costs.setValue(current.getLabel(), neighbor.getLabel(), ADJACENT_COST);
					}
				}
			}
		}
	}
	
	public void printCosts(){
		costs.printCostTable();
	}
	
	public void printOpenList(){
		
		DstarNode[] list = new DstarNode[openList.size()];
		
		list = openList.toArray(list);
		
		Arrays.sort(list);
		
		System.out.println("Open List Size: " + openList.size());
		System.out.println("List Head\t\tK\t\tH");
		System.out.println("------------------------------------------------");
		DstarNode node = openList.peek();
		System.out.println(node.getLabel() + "\t\t\t" + String.format("% ,.1f", node.getK()) + "\t\t" + String.format("% ,.1f", node.getH()));
		
		System.out.println("List Head\t\tK\t\tH");
		System.out.println("------------------------------------------------");
		for(DstarNode n: list){
			System.out.println(n.getLabel() + "\t\t\t" + String.format("% ,.1f", n.getK()) + "\t\t" + String.format("% ,.1f", n.getH()));
		}
	}
	
	/**
	 * checks to see of the row,column in in the map
	 * 
	 * TODO get rid of this, and make the same method in DstarMap a public method
	 * 
	 * @param row
	 * @param column
	 * @return
	 */
	private boolean neighborOutOfBounds(int row, int column){
		
		boolean outofbounds = false;
		if(row < 0 || row >= map.getRows() || column < 0 || column >= map.getColumns()){
			outofbounds = true;
		}
		
		return outofbounds;
	}
	
	
	
}
