 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * TableWidget.h
 *
 *  Created on: January 18, 2017
 *      Author: Diana Koenraadt
 */

#ifndef TABLEWIDGET_H
#define TABLEWIDGET_H

#include <QtGui>
#include <QStandardItemModel>
#include <QStandardItem>
#include <qtablewidget.h>
#include <qtreeview.h>
#include <qwidget.h>

// Internal:
#include "int/widgets/Widget.h"
#include "int/widgets/Table/TableViewModel.h"

class GameSignalAdapter;
class TableWidgetGameSignalSubscriber;

class TableWidget : public QTreeView, public Widget<TableWidgetGameSignalSubscriber, TableWidget>
{
        Q_OBJECT
friend TableWidgetGameSignalSubscriber;

public:
    TableWidget(QWidget* parent = 0);
    void initialize(const int nrColumns);

    virtual void saveState() override;
    virtual void restoreState() override;

private:
    TableViewModel* _model = NULL;
};

/*
* Class that handles subscriber data for the TableWidget
* Use delegation to avoid diamond of death on QObject
*/
class TableWidgetGameSignalSubscriber : public QObject, public WidgetGameSignalSubscriber<TableWidget>
{
        Q_OBJECT
public:
    using WidgetGameSignalSubscriber::WidgetGameSignalSubscriber;
    virtual void subscribe(GameSignalAdapter* gameSignalAdapter) override;

private:

    template <typename TType>
    void onValue(uint8_t senderRobotId, std::string category, std::string key, TType value)
    {
        _widget->_model->setData<TType>(category, key, (int)senderRobotId - 1, value);
    }

public Q_SLOTS:
    virtual void onValue(uint8_t senderRobotId, std::string category, std::string key, float value) override;
    virtual void onValue(uint8_t senderRobotId, std::string category, std::string key, bool value) override;
    virtual void onValue(uint8_t senderRobotId, std::string category, std::string key, std::string value) override;
    virtual void clearColumn(uint8_t robotId);
};

#endif // TABLEWIDGET_H
