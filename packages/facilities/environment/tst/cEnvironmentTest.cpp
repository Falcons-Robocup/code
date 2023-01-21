// Copyright 2015-2022 Michel Koenen (Falcons)
// SPDX-License-Identifier: Apache-2.0
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

	cEnvironmentField::getInstance().loadConfig("cEnvironment12x18");
	cEnvironmentField::getInstance().getFieldPOI( P_OWN_CORNER_LEFT, myPoi );
	EXPECT_FLOAT_EQ( -6.0f, myPoi.x);  //test will only work with official field values
	EXPECT_FLOAT_EQ( -9.0f, myPoi.y);
}

TEST(TestSuiteEnvironmentgetAreaPOI, fieldArea)
{
	areaInfo myArea;

	cEnvironmentField::getInstance().loadConfig("cEnvironment12x18");
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


