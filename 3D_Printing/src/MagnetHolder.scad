

//defining some constant
magnet_radius = 3.1;
magnet_height = 4.2;
rod_radius = 4;
base_height_1 = 4;
base_height_2 = 2;
fac = 0.5; //45 deg inclination: easy for printing

module magnet(){
    cylinder(h = magnet_height, r = magnet_radius, center = true, $fn = 100);
}

module base(){
    union(){
        translate([0, 0, 0.5 * (base_height_1 + base_height_2)])
        cylinder(h = base_height_2, r1 = rod_radius, r2 = rod_radius + fac * base_height_2, center = true, $fn = 100);
        cylinder(h = base_height_1, r1 = 0.9 * rod_radius, r2 = 1.01 * rod_radius, center = true, $fn = 100);
    }
}

module holder_vertical(magnet_displacement){
difference(){
    base();
    #hull(){
        translate([0, 0, magnet_radius + magnet_displacement])
        rotate([90, 0, 0])
        magnet();
        translate([0, 0, magnet_displacement])
        rotate([90, 0, 0])
        magnet();
    }
}   
}
module holder_horizontal(magnet_displacement){
difference(){
    base();
    #hull(){
        translate([0, 0, magnet_radius + magnet_displacement])
        magnet();
        translate([0, 0, magnet_displacement])
        magnet();
    }
}   
}


holder_vertical(1.3);
//translate([15, 0, 0])
//holder_vertical(1.7);
//translate([0, 15, 0])
//holder_horizontal(1.3);
//translate([15, 15, 0])
//holder_horizontal(1.7);


