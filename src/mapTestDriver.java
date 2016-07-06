import dstar.DstarMap;
import dstar.DstarPathFinder;

/**
 * 
 * @author J. Levy, P. Shafer
 * 
 * Due: 12/19/2014
 * 
 * Main application driver for D* Implementation
 *
 */
public class mapTestDriver {

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		
		// create and initialize the map
		DstarMap map = new DstarMap(7,7);
		// load a map
		map.loadMap("src/map2.txt");

		// instantiate the DstarPathFinder object
		DstarPathFinder pathFinder = new DstarPathFinder(map);
		
		// start traversing
		pathFinder.traverseMap();
		
	}

}
