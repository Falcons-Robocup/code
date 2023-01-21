// Copyright 2020-2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "widget.hpp"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Widget w;
    w.show();
    return a.exec();
}
