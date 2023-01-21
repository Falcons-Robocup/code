// Copyright 2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef DEWARPER_HPP
#define DEWARPER_HPP

#include <QThread>

#include <cstdio>
#include <string>
#include <vector>

#include "config.hpp"
#include "diagnostics.hpp"
#include "exporter.hpp"

class mainWidget; // forward declaration

class backend : public QThread {
   Q_OBJECT

public:
   explicit backend(mainWidget *parent = nullptr);
   ~backend();

public slots:
   void init();
   void process(size_t cam);
   bool getBusy() {
      return busy;
   }

signals:
   void finished(); // used to cleanup thread, is required because used as signal

private:
   void printObjectsInterval();
   void run() override; // -> run once when ->start() is called

   diagnostics *diagnstcs = nullptr;
   exporter *exprt = nullptr;
   mainWidget *parent = nullptr;
   bool busy;
};

#endif // DEWARPER_HPP
