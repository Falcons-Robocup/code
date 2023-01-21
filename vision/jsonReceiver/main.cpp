// Copyright 2020 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "widget.hpp"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    app.setApplicationName("json receiver");
    app.setApplicationVersion("20210624");

    QCommandLineParser parser;
    parser.setApplicationDescription("  - read json from port 8070\n  - show information in GUI\n  - forward information to mlAdapter (multicast)");
    parser.addHelpOption();
    parser.addVersionOption();

    QCommandLineOption CmdLineOpt(QStringList() << "c" << "cmdline",
                                  QCoreApplication::translate("main", "disable GUI")
                                  );
    parser.addOption(CmdLineOpt);

    QCommandLineOption robotOpt(QStringList() << "r" << "robot",
                                QCoreApplication::translate("main", "robot number"),
                                QCoreApplication::translate("main", "number"), // value name
                                QCoreApplication::translate("main", "0" )   // default value
                                );
    parser.addOption(robotOpt);

    // exits on error
    parser.process(app);

    const QStringList args = parser.positionalArguments();

    Widget w(nullptr, parser.isSet(CmdLineOpt), parser.value(robotOpt).toUInt());

    if( ! parser.isSet(CmdLineOpt) ) {
        w.show();
    }

    int result = app.exec();
    w.close();

    qDebug() << "INFO    end of main";
    return result;
}
