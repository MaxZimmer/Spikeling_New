//Spikeling version 1

Wall = 2;
tol = 0.1;
TOL = 0.3;
Smoothness = 30;

Base = 10;
Top = 10;

PCB = 1;


if(Base==1){
    Base();
}
if(Top==1){
    Top();
}
if(PCB==1){
    PCB();
}

r_M4 = 3/2;
R_M4 = 4/2;
r_M4_Nut = 4/2;
h_M4_Nut = 1;

x_PCB = 96.19;
y_PCB = 78.73;
z_PCB = 1;

h_BNC = 11;
r_BNC = 9.7/2;
x_BNC = 12;
z_BNC = 2.25;
h_bnc = 11.1;
r_bnc = 2/2;
z_bnc = 6.4;
pos_x_Syn1 = -x_PCB/2+80.35;
pos_y_Syn1 = -y_PCB/2+3.1;
pos_x_Syn2 = pos_x_Syn1;
pos_y_Syn2 = -y_PCB/2+18.3;
pos_x_Aout = pos_x_Syn1;
pos_y_Aout = -y_PCB/2+33.5;
pos_x_Din = pos_x_Syn1;
pos_y_Din = -y_PCB/2+48.8;
pos_x_Dout = pos_x_Syn1;
pos_y_Dout = -y_PCB/2+64;

x_pot = 12;
y_pot = 10;
z_pot = 7;
r_pot = 6/2;
h_pot = 13;
pos_x_Vm = -x_PCB/2+3.3;
pos_y_Vm = -y_PCB/2+22.65;
pos_x_Noise = pos_x_Vm;
pos_y_Noise = -y_PCB/2+14.5+22.65;
pos_x_gain1 = pos_x_Vm;
pos_y_gain1 = -y_PCB/2+28.15+22.65;
pos_x_gain2 = pos_x_Vm;
pos_y_gain2 = -y_PCB/2+42+22.65;

r_LED_Base = 10.0/2;
h_LED_Base = 2.1;
r_LED = 8.9/2;
h_LED = 11.2;
h_LED_Top = 3;
pos_x_LED = -x_PCB/2+72.35;
pos_y_LED = -y_PCB/2+9;

r_photo_base = 6/2;
r_photo = 5/2;
h_photo = 8.9;
h_photo_base = 1;
pos_x_photo = -x_PCB/2+8.89;
pos_y_photo = -y_PCB/2+10.25;

x_Arduino = 18;
y_Arduino = 43.5;
z_Arduino = 12.6;
pos_x_Arduino = -x_PCB/2+55.75;
pos_y_Arduino = -y_PCB/2+36;

x_Battery = 29.6;
y_Battery = 53.5;
z_Battery = z_Arduino;
pos_x_Battery = -x_PCB/2+21.75;
pos_y_Battery = -y_PCB/2+21.5;

r_Button = 12/2;
h_Button = 14.5;
pos_x_Button = -x_PCB/2+24.5;
pos_y_Button = -y_PCB/2+10.25;

r_Buzzer = 22/2;
h_Buzzer = 8;
pos_x_Buzzer = -x_PCB/2+64.75;
pos_y_Buzzer = -y_PCB/2+24.25;

x_Component = 25;
y_Component = 12;
z_Component = 8;
pos_x_Component = -x_PCB/2 + 31.25;
pos_y_Component = -y_PCB/2 + 4;

// Screw
pos_x_Screw = -x_PCB/2-TOL-r_M4-Wall/2;
pos_y_Screw = -y_PCB/2-TOL-r_M4-Wall/2;
pos_x2_Screw = x_PCB/2+TOL+r_M4+Wall/2;
pos_y2_Screw = y_PCB/2+TOL+r_M4+Wall/2;

pos_x_screw = 10;
pos_y_screw = 10;
pos_x2_screw = 20;
pos_y2_screw = 20;

module Base(){
difference(){
    union(){
        minkowski(){
            translate([-x_PCB/2,-y_PCB/2,0])cube([x_PCB,y_PCB,Wall]);
            cylinder(r=2*Wall+2*r_M4+TOL,h=Wall,$fn=Smoothness);
        }
        Corners();
    }
    
    difference(){
        union(){
            translate([-x_PCB/2-TOL,-y_PCB/2-TOL,2*Wall-z_PCB-tol])cube([x_PCB+2*TOL,y_PCB+2*TOL,z_PCB+2*tol]);
            translate([-x_PCB/2+Wall,-y_PCB/2+Wall,z_PCB])cube([x_PCB-2*Wall,y_PCB-2*Wall,Wall+tol]);
    Screw_Holes();
        }
        PCB_Platforms();
    }
    PCB_Platform_Nuts();
}
}

module Top(){
translate([0,0,2*Wall+tol])difference(){
    union(){
        minkowski(){
            translate([-x_PCB/2,-y_PCB/2,0])cube([x_PCB,y_PCB,Wall/2]);
            cylinder(r=2*Wall+2*r_M4+TOL,h=Wall/2,$fn=Smoothness);
        }
        minkowski(){
            translate([-x_PCB/2,-y_PCB/2,Wall])cube([x_PCB,y_PCB,Wall]);
            cylinder(r1=2*Wall+2*r_M4+TOL,h=Wall,$fn=Smoothness);
        }
        Corners();
        PCB_Case();
    }
    translate([0,0,-2*Wall-tol])PCB();
    translate([pos_x_Buzzer,pos_y_Buzzer,-2*Wall-tol+h_Buzzer])Buzzer_Case();
    Screw_Holes();
    Screw_Holes_Top();
}
}



module Corners(){
    translate([pos_x_Screw,pos_y_Screw,0])cylinder(r=r_M4+Wall,h=2*Wall,$fn=Smoothness);
    translate([pos_x_Screw,pos_y2_Screw,0])cylinder(r=r_M4+Wall,h=2*Wall,$fn=Smoothness);
    translate([pos_x2_Screw,pos_y2_Screw,0])cylinder(r=r_M4+Wall,h=2*Wall,$fn=Smoothness);
    translate([pos_x2_Screw,pos_y_Screw,0])cylinder(r=r_M4+Wall,h=2*Wall,$fn=Smoothness);
}

module Screw_Holes(){
    translate([pos_x_Screw,pos_y_Screw,0])cylinder(r=r_M4+tol,h=2*Wall+tol,$fn=Smoothness);
    translate([pos_x_Screw,pos_y2_Screw,0])cylinder(r=r_M4+tol,h=2*Wall+tol,$fn=Smoothness);
    translate([pos_x2_Screw,pos_y2_Screw,0])cylinder(r=r_M4+tol,h=2*Wall+tol,$fn=Smoothness);
    translate([pos_x2_Screw,pos_y_Screw,0])cylinder(r=r_M4+tol,h=2*Wall+tol,$fn=Smoothness);
}

module Screw_Holes_Top(){
    translate([pos_x_Screw,pos_y_Screw,Wall])cylinder(r=R_M4+tol,h=2*Wall+tol,$fn=Smoothness);
    translate([pos_x_Screw,pos_y2_Screw,Wall])cylinder(r=R_M4+tol,h=2*Wall+tol,$fn=Smoothness);
    translate([pos_x2_Screw,pos_y2_Screw,Wall])cylinder(r=R_M4+tol,h=2*Wall+tol,$fn=Smoothness);
    translate([pos_x2_Screw,pos_y_Screw,Wall])cylinder(r=R_M4+tol,h=2*Wall+tol,$fn=Smoothness);
}

module PCB_Platforms(){
    difference(){
        union(){
            translate([pos_x_screw,pos_y_screw,0])cylinder(r1=r_M4+2*Wall,r2=r_M4+Wall/2,h=2*Wall-z_PCB-tol,$fn=Smoothness);
            translate([pos_x_screw,pos_y2_screw,0])cylinder(r1=r_M4+2*Wall,r2=r_M4+Wall/2,h=2*Wall-z_PCB-tol,$fn=Smoothness);
            translate([pos_x2_screw,pos_y2_screw,0])cylinder(r1=r_M4+2*Wall,r2=r_M4+Wall/2,h=2*Wall-z_PCB-tol,$fn=Smoothness);
            translate([pos_x2_screw,pos_y_screw,0])cylinder(r1=r_M4+2*Wall,r2=r_M4+Wall/2,h=2*Wall-z_PCB-tol,$fn=Smoothness);
        }
            translate([pos_x_screw,pos_y_screw,0])cylinder(r=r_M4+tol,h=2*Wall-z_PCB,$fn=Smoothness);
            translate([pos_x_screw,pos_y2_screw,0])cylinder(r=r_M4+tol,h=2*Wall-z_PCB,$fn=Smoothness);
            translate([pos_x2_screw,pos_y2_screw,0])cylinder(r=r_M4+tol,h=2*Wall-z_PCB,$fn=Smoothness);
            translate([pos_x2_screw,pos_y_screw,0])cylinder(r=r_M4+tol,h=2*Wall-z_PCB,$fn=Smoothness); 
    }
}

module PCB_Platform_Nuts(){
    translate([pos_x_screw,pos_y_screw,0])cylinder(r=r_M4_Nut+tol,h=h_M4_Nut,$fn=6);
    translate([pos_x_screw,pos_y2_screw,0])cylinder(r=r_M4_Nut+tol,h=h_M4_Nut,$fn=6);
    translate([pos_x2_screw,pos_y2_screw,0])cylinder(r=r_M4_Nut+tol,h=h_M4_Nut,$fn=6);
    translate([pos_x2_screw,pos_y_screw,0])cylinder(r=r_M4_Nut+tol,h=h_M4_Nut,$fn=6);
}


module PCB()color([0,1,0])translate([0,0,2*Wall-z_PCB]){
    translate([-x_PCB/2,-y_PCB/2,0])cube([x_PCB,y_PCB,z_PCB]);
    
    translate([0,0,z_PCB]){     
        translate([pos_x_Syn1,pos_y_Syn1,0])BNC();
        translate([pos_x_Syn2,pos_y_Syn2,0])BNC();
        translate([pos_x_Aout,pos_y_Aout,0])BNC();
        translate([pos_x_Din,pos_y_Din,0])BNC();
        translate([pos_x_Dout,pos_y_Dout,0])BNC();
        
        translate([pos_x_Vm,pos_y_Vm,0])Potentiometer();
        translate([pos_x_Noise,pos_y_Noise,0])Potentiometer();
        translate([pos_x_gain1,pos_y_gain1,0])Potentiometer();
        translate([pos_x_gain2,pos_y_gain2,0])Potentiometer();
        
        translate([pos_x_Arduino,pos_y_Arduino,0])Arduino();
        
        translate([pos_x_Battery,pos_y_Battery,0])Battery();
        
        translate([pos_x_Buzzer,pos_y_Buzzer,0])Buzzer();
        
        translate([pos_x_photo,pos_y_photo,0])Photo_Diode();
        
        translate([pos_x_Button,pos_y_Button,0])Button();
        
        translate([pos_x_Component,pos_y_Component,0])Components();
        
        translate([pos_x_LED,pos_y_LED,0])LED();
    }  
}

module PCB_Case(){
    translate([0,0,3*Wall]){
        translate([pos_x_Battery-Wall/2,pos_y_Battery-Wall/2,0])Battery_Case();
        translate([pos_x_Arduino-Wall/2,pos_y_Arduino-Wall/2,0])Arduino_Case();
        
    }
}

module BNC(){
    cube([x_BNC,x_BNC,z_BNC]);
    translate([x_BNC/2,x_BNC/2,z_BNC])cylinder(r=r_BNC,h=h_BNC,$fn=Smoothness);
    translate([x_BNC/2-h_bnc/2,x_BNC/2,z_BNC+z_bnc])rotate([0,90,0])cylinder(r=r_bnc,h=h_bnc,$fn=Smoothness);
}

module Potentiometer(){
    cube([x_pot,y_pot,z_pot]);
    translate([x_pot/2,y_pot/2,0])cylinder(r=r_pot,h=h_pot,$fn=Smoothness);
}

module Arduino(){
    cube([x_Arduino,y_Arduino,z_Arduino+5*Wall]);
}

module Arduino_Case(){
    minkowski(){
        translate([Wall/2,Wall/2,0])cube([x_Arduino,y_Arduino,z_Arduino-3*Wall]);
        cylinder(r=Wall,h=Wall,$fn=Smoothness);
    }
    minkowski(){
        translate([Wall/2,Wall/2,z_Arduino-2*Wall])cube([x_Arduino,y_Arduino,Wall]);
        cylinder(r1=Wall,h=Wall,$fn=Smoothness);
    }
}

module Battery(){
    cube([x_Battery,y_Battery,z_Battery+5*Wall]);
}
module Battery_Case(){
    minkowski(){
        translate([Wall/2,Wall/2,0])cube([x_Battery,y_Battery,z_Battery-3*Wall]);
        cylinder(r=Wall,h=Wall,$fn=Smoothness);
    }
    minkowski(){
        translate([Wall/2,Wall/2,z_Battery-2*Wall])cube([x_Battery,y_Battery,Wall]);
        cylinder(r1=Wall,h=Wall,$fn=Smoothness);
    }
}

module Buzzer(){
    cylinder(r=r_Buzzer,h=h_Buzzer,$fn=Smoothness);
}

module Buzzer_Case(){
    cylinder(r1=r_Buzzer,r2=r_Buzzer+Wall,h=5*Wall-h_Buzzer+tol,$fn=Smoothness);
}

module Photo_Diode(){
    cylinder(r=r_photo_base,h=h_photo_base,$fn=Smoothness);
    translate([0,0,h_photo_base])cylinder(r=r_photo,h=h_photo,$fn=Smoothness);
}

module Button(){
    cylinder(r=r_Button,h=h_Button,$fn=Smoothness);
}

module Components(){
    cube([x_Component,y_Component,z_Component]);
}

module LED(){
    cylinder(r=r_LED_Base,h=h_LED_Base,$fn=Smoothness);
    translate([0,0,h_LED_Base])cylinder(r=r_LED,h=h_LED,$fn=Smoothness);
}

