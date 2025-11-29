// grab this from https://github.com/SebiTimeWaster/Chamfers-for-OpenSCAD
include <Chamfer.scad>;

$fa = 0.1;
$fs = 0.1;

module switch_cutout() {
	color("red")
		cube([1.5,13.9,13.9],center=true);
	color("blue")
		translate([-15/2,0,0])
			cube([15,15,15],center=true);
	color("green")
		translate([10.7,0,0])
			cube([20,15,15],center=true);
}

module switch_cutout_footprint() {
	union() {
		translate([0,0,-10])
			switch_cutout();
		translate([0,5,6])
			switch_cutout();
		translate([0,8,32-7])
			switch_cutout();
		translate([0,0,48])
			switch_cutout();
		translate([-11.5,2,32.5])
			cube([7,20,100], true);
		translate([-11.5,-25,70])
			cube([7,50,45], true);
	}
}

module joystick() {
	intersection() {
		union() {
			cube([34,27,18]);
			translate([7,1,-3])
				cube([19,25,34]);
			translate([3.1,3.1,-10])
				cylinder(35,1.5,1.5);
			translate([3.1,26.5-3.1,-10])
				cylinder(35,1.5,1.5);
			translate([34-3.6-1.1,3.1,-12])
				cylinder(40,1.5,1.5);
			translate([34-3.6-1.1,26.5-3.1,-12])
				cylinder(40,1.5,1.5);
			translate([34/2,26.5/2,18])
				cylinder(18,10,10);
			translate([14,7,-3])
				cube([20,14,5]);
		}
		translate([-10,-15,-10])
			cube(50);
	}
}

module attachies() {
	difference() {
		union() {
			translate([5,0,0])
				cube([75,40,13]);
			intersection() {
				translate([5,-30,0])
					cube([75,80,13]);
				translate([-40,1,-1])
					rotate([45,0,0])
						cube([120,80,80]);
			}
			translate([-25,5,2.4])
				rotate([0,45,0])
					cube([19,35,8]);
		}
		translate([5,10,1])
			rotate([0,-45,0])
				cube(40);
		translate([73,-30,-20])
			rotate([0,-30,0])
				cube(80);
	}
}

difference() {
	union() {
		difference() {
			translate([0,0,-18])
				chamferCube([25,40,140], [[0, 0, 0, 0], [0, 0, 0, 0, [1, 1, 1, 1]]], 2);
			translate([-.8,-10,90])
				cube([26,80,80]);
		}
		translate([14,0,86])
			attachies();
		translate([11,0,86])
			mirror([1,0,0])
				attachies();
		mirror([1,0,0])
			translate([-3,8,77])
				rotate([0,-45,0])
					cube([14,25,6]);
		translate([65,-15,73])
			cube([5,55,15]);
	}
	translate([24.31,10,22])
		switch_cutout_footprint();
	translate([-.8,35,80.6])
		rotate([0,0,-90])
			joystick();
}
