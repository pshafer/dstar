/**
 * @author J. Levy, P. Shafer
 * Due 12/19/2014
 * 
 * Provides an DstarMap object
 *
 */
package dstar;

import java.io.File;
import java.io.FileInputStream;
import java.util.ArrayList;


public class DstarMap {
	
	// number of columns and rows for this map
	private int rows = 0;
	private int columns = 0;
	
	// provides start, goal, and robotLocation states
	private DstarNode start;
	private DstarNode goal;
	private DstarNode robotLocation;
	
	// map is implemented and a 2D array of DstarNodes
	private DstarNode[][] map; 
	
	/**
	 * Builds a map with the given rows and columns
	 *  
	 * @param rows number of rows
	 * @param columns number of columns
	 */
	public DstarMap(int rows, int columns) {
		this.rows = rows;
		this.columns = columns;
		
		map = new DstarNode[rows][columns];
	}
	
	/**
	 * Returns the number of rows in the map
	 * 
	 * @return
	 */
	public int getRows() {
		return rows;
	}
	
	/**
	 * Returns number of columns in the map
	 * @return
	 */
	public int getColumns() {
		return columns;
	}

	
	/**
	 * Gets the DstarNode at postion (row,column)
	 * @param row
	 * @param column
	 * @return
	 */
	public DstarNode getNode(int row, int column){
		return map[row][column];
	}
	
	/**
	 * Returns the start state of the map
	 * 
	 * @return
	 */
	public DstarNode getStart() {
		return start;
	}

	/**
	 * @deprecated
	 * 
	 * Sets the start state at a given coordinate
	 * @param x
	 * @param y
	 */
	public void setStart(int x, int y) {
		this.start = getNode(x,y);
	}

	/**
	 * Returns the goal state
	 * 
	 * @return
	 */
	public DstarNode getGoal() {
		return goal;
	}

	/**
	 * @deprecated
	 * 
	 * Sets the goal state at x,y
	 * 
	 * @param x
	 * @param y
	 */
	public void setGoal(int x, int y) {
		this.goal = getNode(x,y);
		this.goal.setK(0);
		this.goal.setH(0);
	}
	
	/**
	 * Returns the local know node where the robot is located
	 * 
	 * @return the node where the robot is located
	 */
	public DstarNode getRobotLocation() {
		return robotLocation;
	}

	/**
	 * Sets the robots positions
	 * 
	 * @param robotLocation the node where the robot is located
	 * 
	 */
	public void setRobotLocation(DstarNode robotLocation) {
		this.robotLocation = robotLocation;
	}
	
	/**
	 * Returns list of neighbors for a node at a given position
	 * 
	 * @param row row position of node
	 * @param col column position of node
	 * 
	 * @return list of neighbor nodes
	 */
	public ArrayList<DstarNode> getNeighbors(int row, int col){
		// set top and bottom neighbor row values
		int top = row-1;
		int bottom = row+1;
		
		
		ArrayList<DstarNode> neighborList = new ArrayList<DstarNode>();
		
		// set left and right neighbor column values
		int left = col-1;
		int right = col+1;
		
		// top left adjecent neighbor
		if(!neighborOutOfBounds(top,col)){
			 neighborList.add(getNode(top, col));

		}
		
		// direct top neighbor
		if(!neighborOutOfBounds(bottom, col)){
			neighborList.add(getNode(bottom, col));

		}
		
		// top right neighbor
		if(!neighborOutOfBounds(row,right)){
			neighborList.add(getNode(row, right));

		}
		
		// left direct neighbor
		if(!neighborOutOfBounds(row,left)){
			neighborList.add(getNode(row, left));

		}
		
		// right direct neighbor
		if(!neighborOutOfBounds(top, right)){
			neighborList.add(getNode(top, right));

		}
		
		// bottom left neighbor
		if(!neighborOutOfBounds(bottom,left)){
			neighborList.add(getNode(bottom, left));
		}
		
		// direct bottom neighbor
		if(!neighborOutOfBounds(bottom,right)){
			neighborList.add(getNode(bottom, right));

		}
		
		// bottom right neighbor
		if(!neighborOutOfBounds(top,left)){
			neighborList.add(getNode(top, left));
		}
		
		return neighborList;
	}
	
	/**
	 * Check to see if the given row and column are out of bounds on the map
	 * 
	 * @param row row number to check
	 * @param column column number to check
	 * @return true if either row or column is out of bounds, false if the row and column are valid
	 * 
	 */
	private boolean neighborOutOfBounds(int row, int column){
		
		boolean outofbounds = false;
		if(row < 0 || row >= getRows() || column < 0 || column >= getColumns()){
			outofbounds = true;
		}
		
		return outofbounds;
	}
	
	
	/**
	 * Load map file
	 */
	public void loadMap(String filename){
		int row = 0;
		int column = 0;
		
		File file = new File(filename);
		
		if(file.exists()){
			if(file.isFile() && file.canRead()){
				try{
					FileInputStream in = new FileInputStream(filename);
					int read;
					while((read = in.read()) != -1){
						char c = (char) read;
						if(c == 'O' || c == 'B' || c == 'U' || c == 'S' || c == 'G'){
							DstarNode node = new DstarNode(row,column);
							node.setTag("NEW");
							
							char state = c;
							if(c == 'S'){
								state = 'O';
								start = node;
							}else if(c == 'G'){
								state = 'O';
								goal = node;
							}
							
							node.setState(String.valueOf(state));
							map[row][column] = node;
							column++;
						}else{
							row++;
							column=0;
						}
					}
					in.close();
					robotLocation = start;
				}catch(Exception e){
					
				}
			}else{
				System.err.println(file.getName() + " cannot be read");
				System.exit(0);
			}
		}else{
			System.err.println("File Not Found...");
			System.exit(0);
		}
		
		
		
	}
	
	/**
	 * Prints the maps current state
	 */
	public void print(){
		
		int cell_column_count = 15;
		int row_column_count = cell_column_count * this.columns;
		
		char space = ' ';
		char row_seperator_char = '-';
		
		String row_seperator = repeatChar(row_seperator_char,row_column_count + 1);
		String cr1; // cell row 1
		String cr2; // cell row 2
		String cr3; // cell row 3
		String cr4; // cell row 4
		String cr5; // cell row 5
		String cr6; // cell row 6
		
		// loop through rows
		for(int i=0; i < map.length; i++){
			// new row, reinitialize cell row strings
			cr1 = "";
			cr2 = "";
			cr3 = "";
			cr4 = "";
			cr5 = "";
			cr6 = "";
			
			// loop through cells and concatenate values to cell row strings
			for(int j=0; j < map[i].length; j++){
				DstarNode node = getNode(i,j);
				
				String startOrGoal = " ";
				if(node == start){
					startOrGoal = "S";
				}else if(node == goal){
					startOrGoal = "G";
				}
				
				String robot = "   ";
				if(node == robotLocation){
					robot = "(*)";
				}
				
				String tag = node.getTag();
				String state = node.getState();
				String label = node.getLabel();
				String hval = "h:" + String.format("% ,.1f", node.getH());
				String kval = "k:" + String.format("% ,.1f", node.getK());
				
				String bp = "";
				if(node.getBackPointer() == null){
					bp = "b: " + repeatChar(space,11);
				}else{
					bp = "b: " + node.getBackPointer().getLabel() + repeatChar(space,9);
				}
				
				cr1 += "|" + tag + repeatChar(space, 14 - tag.length() - state.length()) + state;
				cr2 += "|" + label + repeatChar(space, 14 - label.length());
				cr3 += "|" + hval + repeatChar(space, 14 - hval.length());
				cr4 += "|" + kval + repeatChar(space, 14 - kval.length());
				cr5 += "|" + bp;
				
				cr6 += "|" + repeatChar(space,5) + robot + repeatChar(space,5) + startOrGoal;
				
			}
			// close the row lines with |
			cr1 += "|";
			cr2 += "|";
			cr3 += "|";
			cr4 += "|";
			cr5 += "|";
			cr6 += "|";
			
			System.out.println(row_seperator);
			System.out.println(cr1);
			System.out.println(cr2);
			System.out.println(cr3);
			System.out.println(cr4);
			System.out.println(cr5);
			System.out.println(cr6);;
		}
		
		System.out.println(row_seperator);
	}
	
	private String repeatChar(char c, int num){
		String repeated = "";
		if(num > 0){
			repeated = new String(new char[num]).replace('\0', c);
		}
		
		return repeated;
	}

	

	
	

}
