/*
 * 2-Axis Pan/Tilt Gimbal Frame
 * For 2804 Hollow Shaft Outrunner Brushless Motors + AS5600 Encoders
 * 
 * HOW TO USE:
 *   1. Open this file in OpenSCAD (free: https://openscad.org/downloads)
 *   2. Uncomment ONE of the three lines at the bottom:
 *        base();
 *        yoke();
 *        camera_mount();
 *   3. Press F6 (Render)
 *   4. File → Export as STL
 *   5. Import STL into Bambu Studio
 *   6. Repeat for each part
 *
 * PRINT SETTINGS (Bambu P1S):
 *   Material: PLA or PETG
 *   Layer height: 0.2mm
 *   Infill: 30-40% (gyroid or grid)
 *   Walls: 3 perimeters
 *   Supports: YES for yoke (the arms overhang)
 *   Print yoke upright (arms pointing up)
 *   Print base flat (feet down)
 *   Print camera mount flat
 */

// ============================================================
// MOTOR DIMENSIONS — edit these if your measurements differ
// ============================================================
motor_dia       = 34.5;   // Motor body outer diameter
motor_height    = 15.0;   // Motor body height
bolt_circle     = 16.0;   // Mounting hole center-to-center (diagonal)
num_bolts       = 4;      // Number of mounting holes
screw_dia       = 2.2;    // M2 hole + clearance
screw_depth     = 3.0;    // Max screw depth into motor
hollow_shaft    = 6.5;    // Center hollow opening
magnet_dia      = 8.0;    // AS5600 magnet diameter (if applicable)

// ============================================================
// DESIGN PARAMETERS — tweak fit and strength
// ============================================================
tol             = 0.3;    // Print tolerance (increase if too tight)
wall            = 4.0;    // General wall thickness
$fn             = 64;     // Cylinder smoothness (increase for smoother)

// Derived
motor_r         = motor_dia / 2;
bolt_r          = bolt_circle / 2;
screw_r         = screw_dia / 2;

// ============================================================
// HELPER: Motor bolt hole pattern
// ============================================================
module bolt_holes(depth=10, extra_r=0) {
    for (i = [0:num_bolts-1]) {
        angle = 360 / num_bolts * i + 45; // 45° offset
        translate([bolt_r * cos(angle), bolt_r * sin(angle), 0])
            cylinder(h=depth, r=screw_r + extra_r, center=true);
    }
}

// ============================================================
// HELPER: Motor clamp ring
// ============================================================
module clamp_ring(height=10, inner_tol=0) {
    difference() {
        cylinder(h=height, r=motor_r + wall/2 + tol, center=false);
        translate([0, 0, -1])
            cylinder(h=height+2, r=motor_r + tol + inner_tol, center=false);
    }
}

// ============================================================
// PART 1: BASE
// ============================================================
module base() {
    base_w = 80;
    base_d = 80;
    base_h = 5;
    ring_h = 10;     // Motor retention ring height above base
    standoff_h = 3;  // Motor mount standoff height above base
    
    difference() {
        union() {
            // Main plate
            translate([-base_w/2, -base_d/2, 0])
                cube([base_w, base_d, base_h]);
            
            // Motor retention ring
            translate([0, 0, base_h])
                clamp_ring(height=ring_h);
            
            // Motor mounting standoffs
            for (i = [0:num_bolts-1]) {
                angle = 360 / num_bolts * i + 45;
                translate([bolt_r * cos(angle), bolt_r * sin(angle), base_h])
                    cylinder(h=standoff_h, r=3.5);
            }
        }
        
        // Subtract: motor mounting screw holes (through standoffs and base)
        translate([0, 0, -1])
            bolt_holes(depth=base_h + standoff_h + 4);
        
        // Subtract: center wiring hole
        translate([0, 0, -1])
            cylinder(h=base_h + ring_h + 4, r=hollow_shaft/2 + 1);
        
        // Subtract: cable channel to edge
        translate([-2, 0, base_h - 2])
            cube([4, base_d, 3]);
    }
    
    // Label
    translate([0, -base_d/2 + 3, base_h])
        linear_extrude(0.5)
            text("PAN", size=5, halign="center", font="Liberation Mono:style=Bold");
}

// ============================================================
// PART 2: YOKE (connects pan rotor → tilt motor)
// ============================================================
module yoke() {
    arm_h = 55;         // Arm height
    arm_t = wall + 1;   // Arm thickness (5mm)
    arm_d = 40;         // Arm depth (front-to-back)
    yoke_w = 65;        // Total width
    plate_h = 5;        // Bottom plate thickness
    clamp_depth = 8;    // Motor clamp ring depth (below plate)
    
    motor_z = plate_h + arm_h * 0.55;  // Tilt motor center height
    
    difference() {
        union() {
            // Bottom plate
            translate([-yoke_w/2, -arm_d/2, 0])
                cube([yoke_w, arm_d, plate_h]);
            
            // Motor clamp ring (goes below plate, fits over pan rotor)
            translate([0, 0, -clamp_depth])
                clamp_ring(height=clamp_depth + 1);
            
            // Left arm
            translate([-yoke_w/2, -arm_d/2, plate_h])
                cube([arm_t, arm_d, arm_h]);
            
            // Right arm
            translate([yoke_w/2 - arm_t, -arm_d/2, plate_h])
                cube([arm_t, arm_d, arm_h]);
            
            // Tilt motor mount plate (on inner face of left arm)
            mount_size = 42;
            mount_t = 5;
            translate([-yoke_w/2 + arm_t, -mount_size/2, motor_z - mount_size/2])
                cube([mount_t, mount_size, mount_size]);
            
            // Tilt motor retention ring (horizontal, on left mount plate)
            translate([-yoke_w/2 + arm_t + mount_t, 0, motor_z])
                rotate([0, 90, 0])
                    clamp_ring(height=8);
            
            // Top cross-brace
            brace_h = 6;
            brace_d = 10;
            translate([-yoke_w/2, -brace_d/2, plate_h + arm_h - brace_h])
                cube([yoke_w, brace_d, brace_h]);
            
            // Gussets (triangular reinforcement at arm bases)
            gusset_size = 15;
            // Left arm gussets
            for (sy = [-1, 1]) {
                translate([-yoke_w/2, sy * arm_d/2, plate_h])
                    rotate([90 * (sy > 0 ? 0 : 1), 0, 0])
                        linear_extrude(arm_t)
                            polygon([[0,0], [0, gusset_size], [gusset_size, 0]]);
            }
            // Right arm gussets  
            for (sy = [-1, 1]) {
                translate([yoke_w/2 - arm_t, sy * arm_d/2, plate_h])
                    rotate([90 * (sy > 0 ? 0 : 1), 0, 0])
                        linear_extrude(arm_t)
                            polygon([[0,0], [0, gusset_size], [gusset_size, 0]]);
            }
            
            // Set screw boss on motor clamp
            translate([motor_r + tol + wall/2, -3, -clamp_depth/2 - 2])
                cube([4, 6, 4]);
        }
        
        // Subtract: tilt motor mounting holes (through left mount plate)
        translate([-yoke_w/2 - 1, 0, motor_z])
            rotate([0, 90, 0])
                bolt_holes(depth=arm_t + 12);
        
        // Subtract: center hole in bottom plate for wiring
        translate([0, 0, -clamp_depth - 1])
            cylinder(h=plate_h + clamp_depth + 4, r=hollow_shaft/2 + 1);
        
        // Subtract: set screw hole
        translate([motor_r + tol - 1, 0, -clamp_depth/2])
            rotate([0, 90, 0])
                cylinder(h=wall + 6, r=1.5);  // M3 set screw
        
        // Subtract: weight reduction holes in arms (optional)
        for (zh = [plate_h + 20, plate_h + 35]) {
            translate([-yoke_w/2 - 1, 0, zh])
                rotate([0, 90, 0])
                    cylinder(h=arm_t + 2, r=6);
            translate([yoke_w/2 - arm_t - 1, 0, zh])
                rotate([0, 90, 0])
                    cylinder(h=arm_t + 2, r=6);
        }
    }
    
    // Labels
    translate([-yoke_w/2 + 1, -arm_d/2 + 3, plate_h + arm_h/2])
        rotate([90, 0, 90])
            linear_extrude(0.5)
                text("TILT→", size=4, halign="center", font="Liberation Mono:style=Bold");
}

// ============================================================
// PART 3: CAMERA MOUNT (attaches to tilt motor rotor)
// ============================================================
module camera_mount() {
    plate_w = 50;
    plate_l = 70;
    plate_t = 5;
    clamp_depth = 8;
    
    // Camera rail dimensions (Logitech C920 clip base ~25mm)
    rail_spacing = 28;
    rail_w = 3;
    rail_h = 4;
    
    difference() {
        union() {
            // Main plate
            translate([-plate_w/2, -plate_l/3, 0])
                cube([plate_w, plate_l, plate_t]);
            
            // Motor clamp ring (motor goes below)
            translate([0, 0, -clamp_depth])
                clamp_ring(height=clamp_depth + 1);
            
            // Camera rails (raised ridges to clip C920 onto)
            translate([-rail_spacing/2 - rail_w, plate_l/4, plate_t])
                cube([rail_w, 35, rail_h]);
            translate([rail_spacing/2, plate_l/4, plate_t])
                cube([rail_w, 35, rail_h]);
            
            // Front lip (keeps camera from sliding forward)
            translate([-rail_spacing/2 - rail_w, plate_l/4 + 35, plate_t])
                cube([rail_spacing + rail_w*2, rail_w, rail_h]);
            
            // Set screw boss
            translate([motor_r + tol + wall/2, -3, -clamp_depth/2 - 2])
                cube([4, 6, 4]);
        }
        
        // Subtract: center wiring hole
        translate([0, 0, -clamp_depth - 1])
            cylinder(h=plate_t + clamp_depth + 4, r=hollow_shaft/2 + 1);
        
        // Subtract: set screw hole
        translate([motor_r + tol - 1, 0, -clamp_depth/2])
            rotate([0, 90, 0])
                cylinder(h=wall + 6, r=1.5);
        
        // Subtract: mounting holes for 1/4-20 camera thread (optional)
        translate([0, plate_l/3, -1])
            cylinder(h=plate_t + 2, r=3.2);  // 1/4-20 = 6.35mm
        
        // Subtract: zip-tie slots (for cable management)
        for (y = [0, 15]) {
            translate([-plate_w/2 + 5, y, -1])
                cube([3, 1.5, plate_t + 2]);
            translate([plate_w/2 - 8, y, -1])
                cube([3, 1.5, plate_t + 2]);
        }
    }
    
    // Label
    translate([0, -plate_l/3 + 3, plate_t])
        linear_extrude(0.5)
            text("CAM", size=5, halign="center", font="Liberation Mono:style=Bold");
}


// ============================================================
// RENDER ONE PART AT A TIME
// ============================================================
// Uncomment ONE line, press F6 to render, then Export as STL:

base();
// yoke();
// camera_mount();

// Or preview all three together (don't export this — just for visualization):
// color("SteelBlue") base();
// color("Orange") translate([0, 0, 30]) yoke();
// color("LimeGreen") translate([100, 0, 0]) camera_mount();
