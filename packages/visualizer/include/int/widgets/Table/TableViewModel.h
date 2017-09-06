 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * TableViewModel.h
 *
 *  Created on: June 30, 2016
 *      Author: Diana Koenraadt
 */

#ifndef TABLEVIEWMODEL_H
#define TABLEVIEWMODEL_H

#include <QtGui>
#include <QStandardItemModel>
#include <QStandardItem>

/*
* Table ViewModel class
*/
class TableViewModel : public QStandardItemModel
{
        Q_OBJECT

public:
    TableViewModel(const int nrColumns);

    template <typename TType>
    void setData(std::string category, std::string key, int column, TType value)
    {
        setData(category, key, column, QVariant(value));
    }

    void setData(std::string category, std::string key, int column, QVariant value);
    void add(std::string category, std::string key); // Add for display

private:
    int _nrValuesPerKey;
    QList<QStandardItem *> _tableData;

    bool displayValue(std::string category, std::string key); // Whether or not to display values for the given category/key pair.
    std::vector<std::tuple<std::string /* category */, std::string /* key */ >> _toDisplay;
};

#endif // TABLEVIEWMODEL_H
