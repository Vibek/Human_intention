/*****************************************************************************
*                                                                            *
*  //Copyright (c) 2016, Vibekananda Dutta, WUT
  // Faculty of Power and Aeronautical Engineering (MEiL)/ZTMiR Laboratory
  // Warsaw University of Technology
 //  All rights reserved.
*                                                                            *
****************************************************************************/

#include <stdio.h>
#include <iostream>
#include "find_objects.h"
#include "trajectory.h"

void HumanObjectRealtion()
{

	if(nobjects ==1 && object_names == "ipad")
	{
		if(GetDistanceToTargetObject() < 0.2f)
		{
			printf(" Grasping\n");
		}
		elseif (Speed() < 150 && objects_firstframe != objects_currentframe)
		{
			printf("Placing\n");
		}
		else
			printf("No Activity\n");
	}
	
	else if(nobjects ==1 && object_names == "Glass")
	{
		if(GetDistanceToTargetObject() < 0.2f)
		{
		printf(" Grasping\n");
		}
		elseif (GetDistanceToTargetJoint() < 0.2f)
		{
			printf("Drinking\n");
		}
		elseif (Speed() < 150 && objects_firstframe != objects_currentframe)
		{
		printf("Placing\n");
		}					
		else
		printf("No Activity\n");
	}	
}
