// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef PLAYSELECTION_HPP_
#define PLAYSELECTION_HPP_


namespace teamplay
{

enum class PlayEnum {
    INVALID,
    DEFAULT_PLAY
};

class PlaySelection
{
public:
    static PlaySelection& getInstance()
    {
        static PlaySelection instance; // Guaranteed to be destroyed.
                                       // Instantiated on first use.
        return instance;
    }

    teamplay::PlayEnum selectPlay();

private:
    PlaySelection();
    PlaySelection(PlaySelection const&) = delete;
    PlaySelection(PlaySelection &&) = delete;
    PlaySelection operator=(PlaySelection const&) = delete;
    PlaySelection operator=(PlaySelection &&) = delete;
    ~PlaySelection() = default;

    PlayEnum _currentPlay;

};

}

#endif // PLAYSELECTION_HPP_

