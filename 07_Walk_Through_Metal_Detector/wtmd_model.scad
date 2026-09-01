// ============================================================================
// Parametric 3D Model: Walk-Through Metal Detector (WTMD)
// OpenSCAD Reference Project File
// ============================================================================

// Main Parameters
passage_width = 760;
passage_height = 2000;
panel_depth = 580;
panel_width = 120;
panel_height = 2150;
base_thickness = 15;
base_overhang = 50;

console_height = 120;
console_depth = 620;
console_width = passage_width + (2 * panel_width);

module base_plate() {
    color("DimGray")
    translate([-(panel_width + base_overhang)/2, -panel_depth/2, 0])
    cube([panel_width + base_overhang, panel_depth, base_thickness]);
}

module vertical_panel() {
    color("DarkSlateGray")
    translate([-panel_width/2, -panel_depth/2, base_thickness])
    cube([panel_width, panel_depth, panel_height]);
    
    // Decorative LED strip indicator slot
    color("DodgerBlue")
    translate([-panel_width/2 - 1, -20, base_thickness + 200])
    cube([2, 40, panel_height - 400]);
}

module top_console() {
    color("Silver")
    translate([-console_width/2, -console_depth/2, base_thickness + panel_height])
    cube([console_width, console_depth, console_height]);
    
    // Display screen placeholder
    color("Black")
    translate([-60, -console_depth/2 - 1, base_thickness + panel_height + 40])
    cube([120, 2, 50]);
}

module wtmd_assembly() {
    // Left Base & Panel
    translate([-passage_width/2 - panel_width/2, 0, 0]) {
        base_plate();
        vertical_panel();
    }
    
    // Right Base & Panel
    translate([passage_width/2 + panel_width/2, 0, 0]) {
        base_plate();
        vertical_panel();
    }
    
    // Top Control Console Bridge
    top_console();
}

// Render Assembly
wtmd_assembly();
