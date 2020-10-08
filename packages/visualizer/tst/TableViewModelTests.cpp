 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
