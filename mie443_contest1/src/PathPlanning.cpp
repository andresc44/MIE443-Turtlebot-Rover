// go forward until a wall is reached
// turn in slow increments until scan is such that it is parallel to a wall

// move forward, x distance, adjusting for straightness along the way
// track change in magnitude of odom position
// after certain change in distance, 360 scan

// keep running counter of how much we are turning in a direction
// if the counter is above threshold in an amount of time, we are going in circles.
//     try alternate movement pattern (reverse)

// keep growing array of odom locations where scan was run
// cluster locations, identify areas missing

// potentially look ahead at occupancy grid

// switch statement between left turns and right turns at direct wall hits

// alternative

// view location of nearest odom checkpoints, and move away from them if forward path is not clear

// higher cost to go towards spot thats been mapped

// use occupancy grid to help with cost for left and right turns

// assign global variables to determine weight of different factors

// forward with corrections is assumed until edges hit

// function to determine different scan scenarios possible, and assign output value

// hopefully, scanning intermittently should cover inside regions

// have X different states for scan to return

// parameter for max distance from wall before turning

// check with occupancy grid to left and right to determine if we're close to walls

// if within radius of odom location, don't need to stop again to get same data'

// if we choose left and there is wall, do 180



// const Parameters

const float forward_fastV = 0.15;
const float forward_slowV = 0.08;

const float backwardV = -0.05; // magnitude and direction in x axis
const float max_obst_dist = 0.5; // if less than this, we should turn, meters

const float stops_spacing = 0.5; //distance between checkpoints, meters

const long turning_lim = 3000; //amounts of total degrees turned before turning robot around

const uint8_t LR_weight = 10; //weights
const uint8_t occup_weight = 10;
const uint8_t odom_weight = 10;
const uint8_t scan_weight = 10;

const float checkpoint_radius = 0.2; //how close are we to a previous checkpoint
const uint8_t adjust_angle = 15; // how much to adjust robot by when wall detected

//////////////////////////////////////////////////////////
uint8_t scanMode = 0; // list of X states
float dist_travelled = 0; //how much moved since last checkpoint
float last_odom[2] = {0,0}; //location of last point
long turningCount = 0 //accumulation of how much travelled
bool JustRotated = false; //Did we just make a 90 degree turn?
float odom_array[100] = {initialize all zeros}
uint8_t checkpoint_counter = 1;

while true{
    curr_odom = //get odom pointer value
    dist_travelled = distance(last_odom, curr_odom); //distance is function to determine distance between 2 coordinates
    //add criteria to do checkpoint of forward happens uninterreupted
    if dist_travelled > stops_spacing { // checks space since last checkpoint
        last_odom = curr_odom; //reset reference location
        dist_travelled = 0;
        odom_array[checkpoint_counter] = curr_odom; //keeps track of locations of all past checkpoints, leaves first index as 0,0
        checkpoint_counter += 1;
        rotate360(); //scan area
        break
    }

    scanMode = determine_scan_type(); //recognize different possibilities based on scan array, also look at changes in odom

    if (scanMode == slightly tilted){ 
        adjustrotation(); //fix allignment by ~ 15 degrees
        turningCount += rotation; //keep track of turning
        break //rerun loop
    }
    else if (scanMode == front wall) {
        scan_cmd = clearest_path(); //based on clearest path, L or R, function
        occup_cmd = new_frontier(); // based on nearby occupancy grid, function
        odom_cmd = nearest_checkpoint(odom_array, curr_odom);// based on nearby odom checkpoints, function, checks odom_array
        if (JustRotated) { //bad turning decision, we're at a corner and turned into other wall
            rotate180(); //turn around, think about a dead end corridor situation
            //dead end scenario
            turningCount += -decision*180; //if we made the wrong choice before, we want to overwrite that contribution to the count and add accordingly
            JustRotated = false; //reset flag
            break
        }

        if (turningCount > turning_lim) { //we've been turning too much
            rotate180(); //go other direction
            turningCount = 0; //reset flag
        }
        else if (turningCount < -turning_lim) { //same as above in other direction
            rotate180();
            turningCount = 0;
        }

        decision = sign(scan_cmd*scan_weight + occup_cmd*occup_weight + odom_cmd*odom_weight); //weighted decision of costs
        //rather than weight, potentially do steps to see if that area's been explored
        rotate90(decision) // rotate 90 based on decision
        turningCount += rotation; //add rotation to count
        JustRotated = true;
        break
    }
    else {
        if (nearby_obstacles()) {
            Forward(forward_slowV);
        } //do we go fast or slow
        else {
            Forward(forward_fastV);
        }
         
    }
    JustRotated = false;

}
 
