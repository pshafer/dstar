/**
 * @author J. Levy, P. Shafer
 * 
 * CostTable object for use in D* algorithm.  Provides a priority queue base
 * look look up table for path costs between DstarNodes
 */
package dstar;

import java.util.Hashtable;
import java.util.HashMap;

public class CostTable {

	// cost table is stored using a hash table
	private HashMap<String,Double> table;
	
	/**
	 * Default constructor
	 */
	public CostTable(){
		table = new HashMap<String,Double>();
	}
	
	/**
	 * Uses the labels from 2 nodes to build a hash key and stores value
	 * at the hash key.
	 * 
	 * Costs are 2 way, so we only need to store a combination of the key/value once
	 * 
	 * @param node1 label of node for part of hash key
	 * @param node2 lable of node for part of hash key
	 * @param value value to store key
	 */
	public void setValue(String node1, String node2, double value){
		String hash1 = node1+node2;
		
		if(!isSet(node1,node2)){
			table.put(hash1, value);
		}
	}
	
	/**
	 * Updates the value for the derived hash key.  Since costs are 2 way the value 
	 * only needs to be stored once, so the key can be node1+node2 or node2+node1
	 * 
	 * @param node1 label of node for part of hash key
	 * @param node2 lable of node for part of hash key
	 * @param value value to store for key
	 */
	public void updateValue(String node1, String node2, double value){
		String hash1 = node1+node2;
		String hash2 = node2+node1;
		
		
		if(table.containsKey(hash1)){ // if hash1 exits, then update hash1
			table.put(hash1, value);
		}else if(table.containsKey(hash2)){ // else if hash2 exists, then update hash2
			table.put(hash2, value);
		}
	}
	
	/**
	 * Checks to see of the table contains the derived key
	 * 
	 * @param node1 label of node for part of hash key
	 * @param node2 lable of node for part of hash key
	 * @return true if a key exists for node1+node2 or node2+node1 otherwise false
	 */
	private boolean isSet(String node1, String node2){
		String hash1 = node1+node2;
		String hash2 = node2+node1;
		
		boolean exists = false;
		if(table.containsKey(hash1)){
			exists = true;
		}else if(table.containsKey(hash2)){
			exists = true;
		}
		
		return exists;
	}
	
	/**
	 * Returns the value at the derived hash key
	 * 
	 * @param node1 label of node for part of hash key
	 * @param node2 lable of node for part of hash key
	 * @return  value at the given hash key, -1 if it isn't set
	 */
	public double getValue(String node1, String node2){
		String hash1 = node1+node2;
		String hash2 = node2+node1;
		
		double value = -1;
		boolean set = isSet(node1,node2);
		
		if(set){
			if(table.containsKey(hash1)){
				value = table.get(hash1);
			}else if(table.containsKey(hash2)){
				value = table.get(hash2);
			}
		}
		
		return value;
	}
	
	/**
	 * Prints the cost table in no particular order
	 */
	public void printCostTable(){
		int count = 0;
		for(String key: table.keySet()){
			
			double value = table.get(key);
			System.out.println(count +": " + key + "=>" + String.format("% ,.1f", value));
			count++;
		}
	}
}
