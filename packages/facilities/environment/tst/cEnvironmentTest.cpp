 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cEnvionmentTestRAW.cpp
 *
 * 
 *
 *  Created on: Dec 20,2015
 *      Author: Michel Koenen
 */
#include "ext/cEnvironmentField.hpp"
#include <cstdio>
#include <cmath>

// Bring in gtest
#include <gtest/gtest.h>

TEST(TestSuiteEnvironmentgetFieldPOI, fieldPOI)
{
	poiInfo myPoi;

	cEnvironmentField::getInstance().getFieldPOI( P_OWN_CORNER_LEFT, myPoi );
	EXPECT_FLOAT_EQ( -6.0f, myPoi.x);  //test will only work with official field values
	EXPECT_FLOAT_EQ( -9.0f, myPoi.y);
}

TEST(TestSuiteEnvironmentgetAreaPOI, fieldArea)
{
	areaInfo myArea;

	cEnvironmentField::getInstance().getFieldArea( A_FIELD, myArea);
	float width=abs(myArea.R.corner1.x) + abs(myArea.R.corner2.x);
	float length=abs(myArea.R.corner1.y) + abs(myArea.R.corner2.y);


	EXPECT_FLOAT_EQ( 12.0f, width);  //test will only work with official field values
	EXPECT_FLOAT_EQ( 18.0f, length);
}

TEST(TestSuiteEnvironmentAreaTypes, fieldAreaTypes)
{
	areaInfo myArea1,myArea2,myArea3;

	cEnvironmentField::getInstance().getFieldArea( A_FIELD, myArea1);
	cEnvironmentField::getInstance().getFieldArea( A_CENTER_CIRCLE, myArea2);
	cEnvironmentField::getInstance().getFieldArea( A_TEST_TRI, myArea3);


	EXPECT_EQ( myArea1.type, (areaType) rectangle);
	EXPECT_EQ( myArea2.type, (areaType) circle);
	EXPECT_EQ( myArea3.type, (areaType) triangle);
}

TEST(TestSuiteEnvironmentIsPositionInArea, predefinedArea)
{
	bool inArea1=true;
	bool inArea2=false;
	bool inArea3=false;

	inArea1=cEnvironmentField::getInstance().isPositionInArea( 10.0, 10.0, A_TEST_TRI );
	inArea2=cEnvironmentField::getInstance().isPositionInArea( 4.0, 5.0, A_TEST_TRI );
	inArea3=cEnvironmentField::getInstance().isPositionInArea( 0.0, 3.0, A_TEST_TRI );

	EXPECT_FALSE(  inArea1 );
	EXPECT_TRUE(   inArea2 );
	EXPECT_TRUE(   inArea3 );
}

TEST(TestSuiteEnvironmentIsPositionInArea, userdefinedArea)
{
	bool inArea1=true;
	bool inArea2=false;

	areaInfo myArea;
	poiInfo myPoi;

	cEnvironmentField::getInstance().getFieldPOI( P_CENTER_LEFT, myPoi );
	myArea.id="My personal area definition";
	myArea.type= circle;
	myArea.C.center= myPoi;
	myArea.C.radius=1.0f;

	inArea1=cEnvironmentField::getInstance().isPositionInArea( -6.0, 1.1, myArea ); //test will only work with official field values
	inArea2=cEnvironmentField::getInstance().isPositionInArea( -6.0, 0.9, myArea );

	EXPECT_FLOAT_EQ( -6.0f, myPoi.x);  //test will only work with official field values
	EXPECT_FLOAT_EQ( 0.0f, myPoi.y);
	EXPECT_FALSE(  inArea1 );
	EXPECT_TRUE(   inArea2 );
}
// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


