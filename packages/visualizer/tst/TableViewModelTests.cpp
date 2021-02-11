// Copyright 2016-2017 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Integration tests for TableViewModel.
 * TableViewModelTests.cpp
 *
 *  Created on: July 3, 2016
 *      Author: Diana Koenraadt
 */
#include <gtest/gtest.h>

#include "int/widgets/Table/TableViewModel.h"

TEST(TableViewModel, setData_category)
{
    // Arrange
    TableViewModel viewModel(2);
    viewModel.add("category", "something");

    // Act 
    viewModel.setData("category", "something", 0, 12345.1);

    // Assert
    QString result = viewModel.item(0, 0)->text();
    ASSERT_EQ(QString("category"), result);
}

TEST(TableViewModel, setData_key)
{
    // Arrange
    TableViewModel viewModel(2);
    viewModel.add("category", "something");

    // Act 
    viewModel.setData("category", "something", 0, 2000);

    // Assert
    QString result = viewModel.itemFromIndex(viewModel.index(0, 0, viewModel.index(0, 0)))->text();
    ASSERT_EQ(QString("something"), result);
}

TEST(TableViewModel, setData_stringValue)
{
    // Arrange
    TableViewModel viewModel(2);
    viewModel.add("category", "something");

    // Act 
    viewModel.setData("category", "something", 0, "value");

    // Assert
    QString result = viewModel.itemFromIndex(viewModel.index(0, 1, viewModel.index(0, 0)))->text();
    ASSERT_EQ(QString("value"), result);
}

TEST(TableViewModel, setData_doubleValue)
{
    // Arrange
    TableViewModel viewModel(2);
    viewModel.add("category", "something");

    // Act 
    viewModel.setData("category", "something", 0, 2000);

    // Assert
    QString result = viewModel.itemFromIndex(viewModel.index(0, 1, viewModel.index(0, 0)))->text();
    ASSERT_EQ(QString("2000"), result);
}

TEST(TableViewModel, setData_floatValue)
{
    // Arrange
    TableViewModel viewModel(2);
    viewModel.add("category", "something");

    float value = 2000;

    // Act 
    viewModel.setData("category", "something", 0, value);

    // Assert
    QString result = viewModel.itemFromIndex(viewModel.index(0, 1, viewModel.index(0, 0)))->text();
    ASSERT_EQ(QString("2000"), result);
}

/*
 * Main entry
 */
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
