package dstar;

import java.util.ArrayList;

public class DstarNode implements Comparable<DstarNode>{

	// Node Tag
	private String tag = "NEW";
	private String state = "O";
	private String label = "";
	
	private int row;
	private int column;
	
	private double h;	// current cost to goal from this node
	private double k;	// lowest value of h that this node has seen
	//private ArrayList<DstarNode> neighborList; 

	private DstarNode backPointer;
	
	public DstarNode(int row, int column){
		
		// let this column know where it is at
		this.row = row;
		this.column = column;
		
		// concatenate row and column to generate the label
		// label will be used as a key in the cost hashtable
		this.setLabel(Integer.toString(row) + Integer.toString(column));
		
		this.h = 0;
		this.k = 0;
		setBackPointer(null);
		//neighborList = new ArrayList<DstarNode>();
	}

	public DstarNode getBackPointer() {
		return backPointer;
	}

	public void setBackPointer(DstarNode backPointer) {
		this.backPointer = backPointer;
	}

	/*
	public void addNeighbor(DstarNode neighbor){
		neighborList.add(neighbor);
	}
	*/
	
	public int getRow() {
		return row;
	}

	public int getColumn() {
		return column;
	}
	
	public double getH() {
		return h;
	}

	public void setH(double h) {
		this.h = h;
	}

	public double getK() {
		return k;
	}

	public void setK(double k) {
		this.k = k;
	}

	public String getTag() {
		return tag;
	}

	public void setTag(String tag) {
		this.tag = tag;
	}

	public String getState() {
		return state;
	}

	public void setState(String state) {
		this.state = state;
	}
	
	public String getLabel() {
		return label;
	}

	public void setLabel(String label) {
		this.label = label;
	}

	
	/**
	 * Parameters:
	 * o - the object to be compared.
	 * R:
	 * a negative integer, zero, or a positive integer as this object is less than, equal to, or greater than the specified object.
	 */
	public int compareTo(DstarNode o) {

		int compare;
		
		if(this.k < o.k){
			compare = -1;
		}else if(this.k > o.k){
			compare = 1;
		}else{
			compare = 0;
		}
		
		return compare;
	}

	


	
	
}
