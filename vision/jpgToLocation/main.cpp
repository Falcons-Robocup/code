// Copyright 2021-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "mainWidget.hpp"

#include <QApplication>

int main(int argc, char *argv[]) {
   QApplication a(argc, argv);
   mainWidget w;
   w.show();
   return a.exec();
}
