 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * PlaybackWidget.cpp
 *
 *  Created on: August 28, 2018
 *      Author: Jan Feitsma
 */


#include <qboxlayout.h>
#include <QStyle>

#include "tracing.hpp"

// Internal:
#include "int/widgets/Playback/PlaybackWidget.h"

PlaybackWidget::PlaybackWidget(QWidget *parent)
    : QWidget(parent),
      Widget()
{
    QHBoxLayout* layoutH = new QHBoxLayout(this);

    QPushButton *buttonSB = new QPushButton("", this);
    buttonSB->setIcon(style()->standardIcon(QStyle::SP_MediaSeekBackward));
    layoutH->addWidget(buttonSB);
    
    _button1 = new QPushButton("", this);
    _button1->setIcon(style()->standardIcon(QStyle::SP_MediaPause));
    layoutH->addWidget(_button1);

    QPushButton *buttonSF = new QPushButton("", this);
    buttonSF->setIcon(style()->standardIcon(QStyle::SP_MediaSeekForward));
    layoutH->addWidget(buttonSF);
    
    _dropdown = new QComboBox();
    _dropdown->addItem("0.1x", QVariant(0.10));
    _dropdown->addItem("0.2x", QVariant(0.20));
    _dropdown->addItem("0.5x", QVariant(0.50));
    _dropdown->addItem("normal", QVariant(1.00)); // default index 3
    _dropdown->addItem("2x", QVariant(2.0));
    _dropdown->addItem("5x", QVariant(5.0));
    _dropdown->addItem("10x", QVariant(10.0));
    _dropdown->setCurrentIndex(3); // default index 3
    layoutH->addWidget(_dropdown);

    _slider = new QSlider(Qt::Horizontal, this);
    _slider->setRange(0, (int)SLIDER_RESOLUTION);
    _slider->setValue(_slider->maximum());
    layoutH->addWidget(_slider, 1.0);

    connect(_button1, SIGNAL(released()), this, SLOT(onButton1Change()));
    connect(_slider, SIGNAL(valueChanged(int)), this, SLOT(onSliderValueChange(int)));
    connect(buttonSB, SIGNAL(released()), this, SLOT(stepBack()));
    connect(buttonSF, SIGNAL(released()), this, SLOT(stepForward()));
    connect(_dropdown, SIGNAL(currentIndexChanged(int)), this, SLOT(onSpeedChange(int)));
}

void PlaybackWidget::registerPlaybackControl(PlaybackControl *pb)
{
    TRACE_FUNCTION("");
    _pbControl = pb;
    // register the slider in playbackControl so it can automatically update it, if applicable
    _pbControl->registerWidget(this);
}

void PlaybackWidget::stepBack()
{
    TRACE_FUNCTION("");
    if (_pbControl != NULL)
    {
        _pbControl->stepBack();
    }
}

void PlaybackWidget::stepForward()
{
    TRACE_FUNCTION("");
    if (_pbControl != NULL)
    {
        _pbControl->stepForward();
    }
}

void PlaybackWidget::onSpeedChange(int idx)
{
    float speed = _dropdown->itemData(idx).toFloat();
    std::ostringstream ss;
    ss << "speed=" << speed;
    TRACE_FUNCTION(ss.str().c_str());
    if (_pbControl != NULL)
    {
        _pbControl->setSpeed(speed);
    }
}

void PlaybackWidget::onButton1Change()
{
    TRACE_FUNCTION("");
    if (_pbControl != NULL)
    {
        _pbControl->togglePause();
        // update icon, similar to how Youtube shows it:
        // * when playing, a button 'pause' (||) is shown
        // * when paused, a button 'resume' (>) is shown
        if (_pbControl->isPaused())
        {
            _button1->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
        }
        else
        {
            _button1->setIcon(style()->standardIcon(QStyle::SP_MediaPause));
        }
    }
}

void PlaybackWidget::onSliderValueChange(int v)
{
    std::ostringstream ss;
    ss << "v=" << v;
    TRACE_FUNCTION(ss.str().c_str());
    if (_pbControl != NULL)
    {
        _pbControl->rseek(v / SLIDER_RESOLUTION);
    }
}

void PlaybackWidget::setSliderValue(int v)
{
    TRACE_FUNCTION("");
    _slider->setValue(v);
}

