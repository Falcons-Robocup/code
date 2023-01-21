// Copyright 2021-2022 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef REFBOX_COMMAND_H_
#define REFBOX_COMMAND_H_

#include <map>
#include <string>
#include <QJsonObject>

namespace rbc
{
    class RefboxCommand
    {
    public:
        using Arguments = std::map<std::string,std::string>;

        RefboxCommand();
        RefboxCommand(std::string command, std::string team, Arguments arguments);
        ~RefboxCommand();

        bool isValid() const;
        std::string command() const;
        std::string target() const;

        bool hasArgument(std::string key) const;
        std::string getArgument(std::string key) const;
        const Arguments& getArguments() const;

        static RefboxCommand fromString(const std::string &str);
        static RefboxCommand fromJson(const QJsonObject &json);

    private:
        bool _isValid;
        std::string _command;
        std::string _target;
        Arguments _arguments;
    };

}
#endif
