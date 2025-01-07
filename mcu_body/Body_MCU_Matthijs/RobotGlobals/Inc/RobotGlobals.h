/*
 * RobotGlobals.h
 *
 *  Created on: Apr 26, 2024
 *      Author: matthijs
 */

#ifndef INC_ROBOTGLOBALS_H_
#define INC_ROBOTGLOBALS_H_

#define UPDATE_20HZ 20
#define UPDATE_16HZ 16
#define UPDATE_10HZ 10
#define UPDATE_5HZ 5

enum ENUM_BodyParts{
    LeftArm,
    RightArm,
	Base,
	LeftBaseMotor,
	CenterBaseMotor,
	RightBaseMotor,
	LeftHead,
	RightHead
};

extern enum ENUM_BodyParts BodyParts;

struct RGBLeds_StateType {
		int tmp;
	};

enum ENUM_Booleans {
	False,
	True
	};

extern enum ENUM_Booleans Boolean;

#endif /* INC_ROBOTGLOBALS_H_ */
