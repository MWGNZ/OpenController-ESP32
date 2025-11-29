$fa = 0.1;
$fs = 0.1;

translate([-1.25,-(5+2.4)/2,0]){
	cube([2.5,1.2,3.5]);
	translate([0,1.2+5,0])
		cube([2.5,1.2,3.5]);
}
translate([0,0,-1.9])
	cylinder(d=40, h=2);
