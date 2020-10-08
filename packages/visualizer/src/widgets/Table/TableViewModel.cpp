 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * TableViewModel.cpp
 *
 *  Created on: July 3, 2016
 *      Author: Diana Koenraadt
 */

#include <boost/range/adaptor/map.hpp>
#include <sstream>
#include "falconsCommon.hpp"
#include <QMetaType>
#include "tracing.hpp"

// Internal:
#include "int/widgets/Table/TableViewModel.h"

TableViewModel::TableViewModel(const int nrColumns)
      : _nrValuesPerKey(nrColumns)
{
    // Floating point exception if the table is empty. Be sure to add at least one (empty) element.
    add("empty", "");
    setData("empty", "", 0, QVariant(""));
    setHorizontalHeaderItem(0, new QStandardItem());

    for (int i = 0; i < _nrValuesPerKey; ++i)
    {
        std::string label;
        std::ostringstream str;
        str << (i + 1);
        setHorizontalHeaderItem(i + 1, new QStandardItem(str.str().c_str()));
    }
}

void TableViewModel::setData(std::string category, std::string key, int column, QVariant value)
{
    if (column >= _nrValuesPerKey || category.empty())
    {
        return;
    }

    if (!displayValue(category, key))
    {
        return;
    }

    // Check if there is already a row for this category. If not, create it
    QStandardItem* categoryItem = 0;
    for (int i = 0; i < this->invisibleRootItem()->rowCount(); ++i)
    {
        categoryItem = this->itemFromIndex(index(i, 0));
        if (categoryItem != 0)
        {
            // Reuse the empty category item
            if (categoryItem->data().toString().toStdString() == "empty")
            {
                categoryItem->setData(QVariant(QString::fromStdString(category)));
                categoryItem->setText(QString::fromStdString(category));
                break;
            }

            if (categoryItem->data().toString().toStdString() == category)
            {
                break;
            }

            categoryItem = 0; // Not found, reset
        }
    }

    if (categoryItem == 0)
    {
        categoryItem = new QStandardItem(QString::fromStdString(category));
        categoryItem->setData(QVariant(QString::fromStdString(category)));

        // Don't just add the key, also add placeholders for the value
        QList<QStandardItem *> rowItems;
        rowItems << categoryItem;
        for (int i = 0; i < _nrValuesPerKey; ++i)
        {
            QStandardItem* item = new QStandardItem("");
            rowItems << item;
        }
        this->invisibleRootItem()->appendRow(rowItems);
        // this->sort(0);
    }

    if (key.empty())
    {
        return;
    }

    // Check if there is already a row for this key. If not, create it. Add empty elements for each column.
    QStandardItem* keyItem = 0;
    for (int i = 0; i < categoryItem->rowCount(); ++i)
    {
        keyItem = this->itemFromIndex(index(i, 0, categoryItem->index()));
        if (keyItem != 0 && keyItem->data().toString().toStdString() == key)
        {
            break;
        }

        keyItem = 0; // Not found, reset
    }

    if (keyItem == 0)
    {
        keyItem = new QStandardItem(QString::fromStdString(key));
        keyItem->setData(QVariant(QString::fromStdString(key)));

        // Don't just add the key, also add placeholders for the value
        QList<QStandardItem *> rowItems;
        rowItems << keyItem;
        for (int i = 0; i < _nrValuesPerKey; ++i)
        {
            QStandardItem* item = new QStandardItem();
            rowItems << item;
        }
        categoryItem->appendRow(rowItems);
        emit dataChanged(index(0, 0, categoryItem->index()), index(0, _nrValuesPerKey, categoryItem->index()));
        emit dataChanged(keyItem->index(), keyItem->index());
    }

    // Find the value for this item (must exist at this point) and set its new data
    QStandardItem* valueItem = this->itemFromIndex(index(keyItem->index().row(), column + 1, categoryItem->index()));
    _toDisplay[std::make_tuple(category, key)] |= valueItem->text().size();
    // TRACE("setData(cat=%s,key=%s,col=%d,value=%s)", category.c_str(), key.c_str(), column, value.toString().toStdString().c_str());
    // Convert floats to strings with 2 decimals; if try for all numerical values also Ints get converted, so convert specific items only. See TRAC ticket #428
    if ((key == "cpuLoad")||(key == "networkLoad")||(key == "eventFreq")||(key == "fps")||(key == "visScore")||(category == "HALMW")) 
    {        
        valueItem->setText(QString("%1").arg(value.QVariant::toDouble(), 0, 'f', 2));
    }
    else
    {    
        valueItem->setText(value.toString());
    }
    emit dataChanged(valueItem->index(), valueItem->index());
    emit layoutChanged();
}

void TableViewModel::add(std::string category, std::string key)
{
    _toDisplay[std::make_tuple(category, key)] = false;
}

bool TableViewModel::displayValue(std::string category, std::string key)
{
    for (auto it = _toDisplay.begin(); it != _toDisplay.end(); ++it)
    {
        if (category.compare(std::get<0>(it->first)) == 0)
        {
            if (key.compare(std::get<1>(it->first)) == 0)
            {
                //TRACE("displayValue(%s,%s)=true", category.c_str(), key.c_str());
                return true;
            }
        }
    }

    //TRACE("displayValue(%s,%s)=false", category.c_str(), key.c_str());
    return false;
}

void TableViewModel::clearColumn(int column)
{
    TRACE("clear column %d", column);
    for (auto it = _toDisplay.begin(); it != _toDisplay.end(); ++it)
    {
        if (it->second)
        {
            setData(std::get<0>(it->first), std::get<1>(it->first), column, QVariant(""));
        }
    }
}


