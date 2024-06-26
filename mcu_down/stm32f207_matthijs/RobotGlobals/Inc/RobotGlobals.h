/*
 * RobotGlobals.h
 *
 *  Created on: Apr 26, 2024
 *      Author: matthijs
 */

#ifndef INC_ROBOTGLOBALS_H_
#define INC_ROBOTGLOBALS_H_

#define UPDATE_10HZ 10
#define UPDATE_5HZ 5

enum ENUM_BodyParts{
    LeftArm,
    RightArm,
	Base
};

enum ENUM_BodyParts BodyParts;

struct RGBLeds_StateType {
		int tmp;
	};

enum ENUM_Booleans {
	False,
	True
	};

enum ENUM_Booleans Boolean;

#endif /* INC_ROBOTGLOBALS_H_ */
