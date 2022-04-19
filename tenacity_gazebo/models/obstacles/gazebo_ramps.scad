

module walled_ramp() { 
difference() { 
cube([5000,2000,2000],center=true);
translate([0,0,500]) rotate([0,10,0])cube([8000,1500,2000],center=true);
}
}
module block() {
    cube([5000,2000,900],center=true);
}

module ramp() { 
    difference() { 
cube([5000,2000,2000],center=true);
translate([0,0,500]) rotate([0,10,0]) cube([8000,3000,2000],center=true);
}
}
module low_walled_ramp() { 
    difference() { 
        walled_ramp();
       translate([0,0,800]) rotate([0,10,0]) cube([8000,3000,2000],center=true);
    }
}
        
walled_ramp();
//translate([-5000,0,-500]) block();
//translate([0,5000,0]) ramp();

//translate([0,-5000,0]) low_walled_ramp();
