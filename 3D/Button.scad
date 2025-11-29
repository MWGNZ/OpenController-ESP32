$fa = 0.1;
$fs = 0.1;

translate([-1.25,-(5+2.3)/2,0]){
	cube([2.5,1.2,3.3]);
	translate([0,1.15+5,0])
		cube([2.5,1.2,3.3]);
}
translate([0,0,0])
	cube([13,13,1], center = true);

