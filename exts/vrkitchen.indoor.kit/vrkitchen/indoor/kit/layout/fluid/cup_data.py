from ..param import ROOT

CUP_ROOT = ROOT + "/3dmodels/cup/"
NEW_CUP_ROOT = ROOT + "/sample/custom/Cup/"

FAUCET_INFO = {
    "1028": {
        "inflow_pos": [-17.4121, 4.63152, 0],
        "joints":[
            "link_2/joint_0",
            "link_2/joint_1",   
        ]
    },
    "148": {
        "inflow_pos": [-17.30, 4.10 , 0],
        "joints":[
            "link_1/joint_0",
        ]
    },
    "149": {
        "inflow_pos": [-10.80, 7.0 , 0],
        "joints":[
            "link_3/joint_0",
            "link_3/joint_1",
        ]
    },

    "153": {
        "inflow_pos": [-13.4587, 7.00 , -2.5],
        "joints":[
            "link_1/joint_0",
        ]
    },

     "154": {
        "inflow_pos": [-7.0, 19.00 , 0.0],
        "joints":[
            "link_2/joint_0",
            "link_2/joint_1",
        ]
    },

    "156": {
        "inflow_pos": [-17.00, 6.00 , 0.0],
        "joints":[
            "link_1/joint_0",
        ]
    },

    "693": {
        "inflow_pos": [-14.3453, -6.21179, -0.20894],
        "joints":[
            "link_2/joint_1",
        ]
    },


    "1034": {
        "inflow_pos": [-17.967, 4.04622, 4.11386],
        "joints":[
            "link_1/joint_0",
        ]
    },
    "1052": {
        "inflow_pos": [-14.8737, 4.21977, 1.06383],
        "joints":[
            "link_2/joint_0",
        ]
    },
    "1053": {
        "inflow_pos": [-9.99254, 1.0, 0],
        "joints":[
            "link_1/joint_0",
        ]
    }

    
}


CUP_PARTICLE_INFO = [
        {
            "usd_path": NEW_CUP_ROOT + "0/cup.usd", 
            "mesh_name": "cupShape",
            #"volume_container": "cup_volume",
            "cylinder_height": 15.0,
            "cylinder_radius": 4.5,
            "particle_offset": [0, 1.05, 0],
            "cup_offset": [0, 0, 0],
            "scale": 1.0
        },
        {
            "usd_path": NEW_CUP_ROOT + "1/cup.usd", 
            "mesh_name": "cupShape",
            "volume_container": "cup_volume",
            "cylinder_height": 15.0,
            "cylinder_radius": 4.5,
            "particle_offset": [0, 1.05, 0],
            "cup_offset": [0, 0, 0],
            "scale": 1.0
        },
        {
            "usd_path": CUP_ROOT + "bottle0.usd", 
            "mesh_name": "D_printable_bottle",
            "cylinder_height": 15.0,
            "cylinder_radius": 4.5,
            "particle_offset": [2.0, 1.05, 0],
            "cup_offset": [0, 0, 0],
            "scale": 0.25
        },
        {
            "usd_path": CUP_ROOT + "bottle1.usd", 
            "mesh_name": "bioshock_salts_bottle_final",
            "cylinder_height": 14.0,
            "cylinder_radius": 3.0,
            "particle_offset": [0.0, -10, -2.7],
            # "particle_offset": [0.0, 0, -5],
            "cup_offset": [0, 2.1, 0],
            # "cup_offset": [0, 0, 0],
            "scale": 5.0
        },

        {
            "usd_path": CUP_ROOT + "mug0.usd", 
            "mesh_name": "geom",
            "cylinder_height": 15.0,
            "cylinder_radius": 3.0,
            "particle_offset": [0.0, 1.05, 0],
            "cup_offset": [0, 0, 0],
            "scale": 1.2
        },
        {
            "usd_path": CUP_ROOT + "mug1.usd", 
            "mesh_name": "SM_mug_2_mesh",
            "cylinder_height": 15.0,
            "cylinder_radius": 3.0,
            "particle_offset": [0.0, 1.05, 0],
            "cup_offset": [0, 0, 0],
            "scale": 1.2
        },
        {
            "usd_path": CUP_ROOT + "jar0.usd", 
            "mesh_name": "mesh",
            "cylinder_height": 18.0,
            "cylinder_radius": 5.0,
            "particle_offset": [0.0, 1.05, 0],
            "cup_offset": [0, 0, 0],
            "scale": 1.2
        },
    ]