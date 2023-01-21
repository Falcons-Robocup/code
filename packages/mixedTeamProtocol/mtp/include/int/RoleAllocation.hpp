// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef _INCLUDED_MIXEDTEAMPROTOCOL_ROLEALLOCATION_HPP_
#define _INCLUDED_MIXEDTEAMPROTOCOL_ROLEALLOCATION_HPP_

// headers from this package
#include "ext/PlayerId.hpp"
#include "ext/Roles.hpp"
#include "int/Errors.hpp"

namespace mtp
{

RoleCount roleAllocationToCount(RoleAllocation const &roles);

struct RoleAllocationAlgorithmInput
{
    PlayerId        myId;
    RoleAllocation  currentRoles;
    std::map<PlayerId, PreferredRole> preferredRoles;

    RoleAllocationAlgorithmInput(PlayerId id) : myId(id) {};
};

class RoleAllocationAlgorithm
{
public:
    // base class
    // derived implementations should only implement _run()
    
    // behavior:
    // * if algorithm succeeds, then:
    //     a) 'result' allocation is according to the specification (min/max per role)
    //     b) own role preference is satisfied (example: force becoming goalkeeper)
    // * otherwise, a nonzero error is set
    // * if current role allocation is OK, then it will be reused
    RoleAllocationAlgorithm(RoleAllocationAlgorithmInput const &input);
    void run(); // cannot be called at construction time, so must be called by client

    // algorithm result
    RoleAllocation result;
    uint8_t error = ERROR_UNINITIALIZED;
    std::string describe() const;

protected:
    RoleAllocationAlgorithmInput _input;
    bool currentIsOk() const;
    void checkAndFillInputs();
    void checkResult();
    virtual void _run() = 0; // to be filled in by derived class

}; // end of base class RoleAllocationAlgorithm


class RoleAllocationAlgorithmBruteForce: public RoleAllocationAlgorithm
{
public:
    // brute force algorithm: generate all candidates and select the best one using a penalty calculation
    // such that the algorithm will prefer staying close to current role allocation
    // (minimize role swaps unless absolutely needed)
    using RoleAllocationAlgorithm::RoleAllocationAlgorithm;

    void _run();

private:
    void check() const;
    std::vector<RoleAllocation> generateCandidates();
    float calculatePenalty(RoleAllocation const &candidate);

}; // end of class RoleAllocationAlgorithmBruteForce

class RoleAllocationAlgorithmKuhnMunkres: public RoleAllocationAlgorithm
{
public:
    // this variant uses the standard Kuhn-Munkres algorithm (also known as Hungarian algorithm)
    // it is slightly less versatile as LinearProgramming, but much more faster and good enough for now
    // it is also much easier to install than Google OR-tools
    using RoleAllocationAlgorithm::RoleAllocationAlgorithm;

    void _run();

private:

}; // end of class RoleAllocationAlgorithmKuhnMunkres

} // end of namespace mtp

#endif
