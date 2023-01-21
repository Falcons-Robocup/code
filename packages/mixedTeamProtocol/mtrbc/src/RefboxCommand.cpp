// Copyright 2021-2022 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include "RefboxCommand.h"
#include <string.h>
#include <QtCore/QJsonDocument>
#include <QStringList>

using namespace std;

namespace rbc
{
    RefboxCommand::RefboxCommand()
        : _isValid(false), _command(""), _target(""), _arguments()
    {
    }

    RefboxCommand::RefboxCommand(std::string command, std::string target, Arguments arguments)
        : _isValid(true), _command(command), _target(target), _arguments(arguments)
    {
    }

    RefboxCommand::~RefboxCommand()
    {
    }

    bool RefboxCommand::isValid() const
    {
        return _isValid;
    }

    std::string RefboxCommand::command() const
    {
        return _command;
    }

    std::string RefboxCommand::target() const
    {
        return _target;
    }

    bool RefboxCommand::hasArgument(std::string key) const
    {
        return _arguments.count(key) != 0;
    }

    std::string RefboxCommand::getArgument(std::string key) const
    {
        return _arguments.at(key);
    }

    const rbc::RefboxCommand::Arguments& RefboxCommand::getArguments() const
    {
        return _arguments;
    }

    RefboxCommand RefboxCommand::fromString(const std::string &str)
    {
        QJsonParseError error;
        const QJsonDocument d = QJsonDocument::fromJson(QString::fromStdString(str).toUtf8(), &error);

        if (d.isNull())
        {
            QMessageLogger().warning("[RefboxProtocol2020_Command] fromString() failure: header invalid JSON, %s", qUtf8Printable(error.errorString()));
            return RefboxCommand();
        }

        if (!d.isObject())
        {
            QMessageLogger().warning("[RefboxProtocol2020_Command] fromString() failure: header JSON document not an object");
            return RefboxCommand();
        }
        const QJsonObject obj = d.object();

        return fromJson(obj);
    }

    RefboxCommand RefboxCommand::fromJson(const QJsonObject &obj)
    {
        const QJsonValue command = obj.value("command");
        if (!command.isString())
        {
            QMessageLogger().warning("[RefboxProtocol2020_Command] fromJson() failure: header missing/invalid 'command' field");
            return RefboxCommand();
        }
        QMessageLogger().info("[RefboxProtocol2020_Command] fromJson() notice: received refbox command '%s'", qUtf8Printable(command.toString()));

        const QJsonValue targetTeam = obj.value("targetTeam");
        if (!targetTeam.isString())
        {
            QMessageLogger().warning("[RefboxProtocol2020_Command] fromJson() failure: header missing/invalid 'targetteam' field");
            return RefboxCommand();
        }

        std::map<std::string,std::string> arguments;
        for (QString key : obj.keys())
        {
            QJsonValue value = obj.value(key);
            if (value.isString())
            {
                arguments[key.toStdString()] = value.toString().toStdString();
            }
            else if (value.isDouble())
            {
                arguments[key.toStdString()] = std::to_string(value.toInt(-1));
            }
            else
            {
                QMessageLogger().warning("[RefboxProtocol2020_Command] fromJson() notice: received unknown refbox argument type '%s'", qUtf8Printable(key));
            }
        }

        return RefboxCommand(command.toString().toStdString(), targetTeam.toString().toStdString(), arguments);
    }
}
