// --- SPECIFICATIONS (Updated for Simulation Alignment) ---
$fn = 60;
wheel_dia = 72;         // [Updated: Match SDF radius 0.036m]
wheel_width = 30;       // [Updated: Match SDF length 0.03m]

// Colors updated to match SDF dark grey/black theme
rim_color = [0.0, 0.0, 1.0]; 
tire_color = [0.1, 0.1, 0.1];

// Hub: Adjusted for larger diameter
hub_r = 12.5;                   
hub_w = 18;                    
hub_offset = -7.0;              

// Rim/Spoke Parameters (Scaled for 72mm diameter)
rim_step1_r = 31;              
rim_step2_r = 27;              
mid_ring_r = 20;               
spoke_count = 24;               
spoke_thickness = 1.5;          

module exact_tread_wheel() {
    
    // 1. REPLICATED TREAD
    difference() {
        union() {
            color(tire_color) cylinder(h=wheel_width, r=wheel_dia/2, center=true);
            
            for (i = [0 : 21]) {
                rotate([0, 0, i * (360 / 22)]) {
                    
                    // ROW 1 & 3: L-SHAPED SHOULDERS
                    // Offset adjusted for 30mm width
                    for (side = [-1, 1]) {
                        translate([wheel_dia/2 - 0.5, 0, side * 10.5]) 
                        union() {
                            cube([2.5, 6, 2], center=true);
                            translate([-1, side * 2.5, 0])
                            cube([4, 1.5, 2], center=true);
                        }
                    }
                    
                    // ROW 2: CENTER CHEVRON
                    rotate([0, 0, 360/44])
                    translate([wheel_dia/2 - 0.5, 0, 0]) {
                        rotate([45, 0, 0]) cube([2, 7, 2], center=true);
                        rotate([-45, 0, 0]) cube([2, 7, 2], center=true);
                    }
                }
            }
        }
        // Hollow out to seat the rim
        cylinder(h=wheel_width + 5, r=rim_step1_r - 0.1, center=true);
    }

    // 2. THE RIM
    color(rim_color) difference() {
        union() {
            cylinder(h=20, r=rim_step1_r, center=true); 
            cylinder(h=14, r=rim_step2_r, center=true); 
        }
        cylinder(h=wheel_width + 10, r=rim_step2_r - 2.5, center=true);
    }

    // 3. SPOKES
    color(rim_color) for (i = [0 : spoke_count - 1]) {
        rotate([0, 0, i * (360 / spoke_count)]) {
            hull() {
                translate([hub_r - 1, 0, hub_offset]) cube([1, spoke_thickness, 8], center=true);
                translate([mid_ring_r, 0, hub_offset/2]) cube([1, spoke_thickness, 10], center=true);
            }
            hull() {
                translate([mid_ring_r, 0, hub_offset/2]) cube([1, spoke_thickness, 10], center=true);
                translate([rim_step2_r - 0.5, 0, 0]) cube([1, spoke_thickness, 12], center=true);
            }
        }
    }

    // 4. THE HUB
    color(rim_color) translate([0, 0, hub_offset]) {
        difference() {
            cylinder(h=hub_w, r=hub_r, center=true);
            // Internal hex or mounting hole
            cylinder(h=hub_w + 2, r=3.1, center=true); 
        }
    }
}
rotate([90, 0, 0])
exact_tread_wheel();