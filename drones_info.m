function [H,Ts,drone1_info, drone2_info] = drones_info
    
    H=6;
    Ts = 0.2;
    
    id1_u1 = 1:H;
    id1_u2 = (1:H) + H;
    id1_x= (1:H) + 2*H;
    id1_z = (1:H) + 3*H;
    id1_theta= (1:H) + 4*H;
    id1_dotx = (1:H) + 5*H;
    id1_dotz= (1:H) + 6*H;
    id1_dottheta = (1:H) + 7*H;
    id2_u1 = (1:H) + 8*H;
    id2_u2 = (1:H) + 9*H;
    id2_x= (1:H) + 10*H;
    id2_z = (1:H) + 11*H;
    id2_theta= (1:H) + 12*H;
    id2_dotx = (1:H) + 13*H;
    id2_dotz= (1:H) + 14*H;
    id2_dottheta = (1:H) + 15*H;
    
    drone1_info=[id1_u1;id1_u2;id1_x;id1_z;id1_theta;id1_dotx;id1_dotz;id1_dottheta];
    
    drone2_info=[id2_u1;id2_u2;id2_x;id2_z;id2_theta;id2_dotx;id2_dotz;id2_dottheta];

   
end

