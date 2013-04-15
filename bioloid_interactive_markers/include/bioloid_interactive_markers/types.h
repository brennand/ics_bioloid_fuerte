#ifndef TYPES_H
#define TYPES_H

typedef struct{
   std::vector<double> pos;
   std::vector<double> vel;
   std::vector<double> eff;
   bool finished;
} Action;


typedef struct {
	double x;
	double y;
	double z;
} Position;

typedef struct {
	double x;
	double y;
	double z;
	double w;
} Orientation;

typedef struct {
	Position position;
	Orientation orientation;
} Pose;

typedef struct{
	Pose left_arm;
	Pose right_arm;
	Pose left_leg;
	Pose right_leg;
} MarkerPose;

#define JR_SAA 0
#define JL_SAA 1
#define JR_SFE 2
#define JL_SFE 3
#define JR_EB 4
#define JL_EB 5
#define JR_HAA 6
#define JL_HAA 7
#define JR_HR 8
#define JL_HR 9
#define JR_HFE 10
#define JL_HFE 11
#define JR_KFE 12
#define JL_KFE 13
#define JR_AFE 14
#define JL_AFE 15
#define JR_AR 16
#define JL_AR 17

#endif // TYPES_H
