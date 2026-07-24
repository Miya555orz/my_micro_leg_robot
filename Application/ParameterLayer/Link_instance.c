#include "Link_instance.h"

/*五连杆信息储存相关*/
Link_Stator_Correction_t My_Link_Stator[Leg_Num];
Link_Coord_t My_Link_Coord[Leg_Num];
Link_Angle_t My_Link_Angle[Leg_Num];
Link_Force_t My_Link_Force[Leg_Num];
Link_Centroid_t My_Link_Centroid[Leg_Num];
Link_Leg_Length_t My_Link_Length[Leg_Num];
Ex_link_data_t Ex_link_data[Leg_Num];

Link_info_t Link_info[Leg_Num] = {
    [R_Leg] = {
        .angle = &My_Link_Angle[R_Leg],
        .coord = &My_Link_Coord[R_Leg],
        .length = &My_Link_Length[R_Leg],
        .force = &My_Link_Force[R_Leg],
        .stator_correction = &My_Link_Stator[R_Leg],
        .centroid = &My_Link_Centroid[R_Leg],
        .Ex_data = &Ex_link_data[R_Leg],
    },
    [L_Leg] = {
        .angle = &My_Link_Angle[L_Leg],
        .coord = &My_Link_Coord[L_Leg],
        .length = &My_Link_Length[L_Leg],
        .force = &My_Link_Force[L_Leg],
        .stator_correction = &My_Link_Stator[L_Leg],
        .centroid = &My_Link_Centroid[L_Leg],
        .Ex_data = &Ex_link_data[L_Leg],
    }
};

Link_t Link[Leg_Num] = {
    [R_Leg] = {
        .info = &Link_info[R_Leg],
        .init = Link_Init,
    },
    [L_Leg] = {
        .info = &Link_info[L_Leg],
        .init = Link_Init,
    }
};
