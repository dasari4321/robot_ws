// ==========================================

// SYMMETRIC ACKERMANN CHASSIS (SWAPPED SLOTS)

// ==========================================

$fn = 80; 


// ---- Core Dimensions ----

plate_length = 270;      

plate_width  = 120;      

plate_thickness = 2;     


// ---- Taper & Notch Parameters ----

front_taper_l = 30; front_nose_w = 60;      

rear_taper_l  = 10; rear_nose_w  = 90;      

notch_l_top = 80; notch_l_bot = 30; notch_depth = 15; notch_offset = 75; 


// ---- Slot Parameters (Orientations Swapped) ----

slot_width = 8;          

// Capped at 40mm to prevent cutting through the 58mm central spine

transverse_len = 40;   

short_len = 20; 


// Module from provided file

module slot(len, wid) { 

    hull() {            

        translate([ -len/2, 0 ]) circle(d=wid);

        translate([ len/2, 0 ])  circle(d=wid);

    }

}


// ==========================================

// MAIN ASSEMBLY

// ==========================================

scale([0.1,0.1,0.1])
{

difference() {

    // 1. RECTANGLE BASE

    linear_extrude(height=plate_thickness, center=true)

    square([plate_length, plate_width], center=true);


    // 2. FOUR TRAPEZIUMS (Exterior Profile)

    for(side_y = [1, -1]) {

        // Front Taper

        translate([plate_length/2, side_y * plate_width/2, 0])

            mirror([0, (side_y == -1) ? 1 : 0, 0])

            linear_extrude(height = plate_thickness + 2, center = true)

                polygon(points=[[0.1, 0.1], [-front_taper_l, 0.1], [0.1, -(plate_width - front_nose_w)/2]]);

        

        // Rear Taper

        translate([-plate_length/2, side_y * plate_width/2, 0])

            mirror([1, (side_y == -1) ? 1 : 0, 0])

            linear_extrude(height = plate_thickness + 2, center = true)

                polygon(points=[[0.1, 0.1], [-rear_taper_l, 0.1], [0.1, -(plate_width - rear_nose_w)/2]]);


        // Side Wheel Clearance Notches

        translate([plate_length/2 - notch_offset, side_y * plate_width/2, 0])

            rotate([0, 0, (side_y == 1) ? 180 : 0])

            linear_extrude(height = plate_thickness + 2, center = true)

                polygon(points=[[-notch_l_top/2, 0], [notch_l_top/2, 0], [notch_l_bot/2, notch_depth], [-notch_l_bot/2, notch_depth]]);

    }


    // 3. INTERNAL ROUNDED CUTS (Swapped Orientation)

    // All slots are now rotated 90 degrees to be "long along width"

    for(side_y = [1, -1]) {

        // Rear Slots (Horizantal) (Legs)

        translate([-plate_length/2 + 25, side_y * 30, 0])

            rotate([0, 0, 90]) 

            linear_extrude(height=plate_thickness + 1, center=true)

                slot(short_len, slot_width*2);        
        

        // Rear Mounting Slots (Horizantal) (Waist)

        translate([-plate_length/2 + 60, side_y * 20, 0])

            rotate([0, 0, 90]) 

            linear_extrude(height=plate_thickness + 1, center=true)

                slot(transverse_len, slot_width);

        // Mid Accessory Slots (Vertical) (Hands)

        translate([plate_length/4-105, side_y * 36, 0])


            linear_extrude(height=plate_thickness + 1, center=true)

                slot(short_len*1.5, slot_width*1.5);


        // Mid Weight Reduction Slots (Horizantal) (Neck)

        translate([5, side_y * 10, 0])

            rotate([0, 0, 90]) 

            linear_extrude(height=plate_thickness + 1, center=true)

                slot(transverse_len, slot_width);        


    // Front Eyes (Vertical)
        translate([plate_length/4+15, side_y * 30, 0]) 


            linear_extrude(height=plate_thickness + 1, center=true)

                slot(short_len, slot_width);


    }

    

    // 4. Central Spine Slot (Mouth)

    translate([40, 0, 0])

        rotate([0, 0, 90]) 

        linear_extrude(height=plate_thickness + 1, center=true)

           slot(transverse_len*0.3, slot_width*4);
}
}