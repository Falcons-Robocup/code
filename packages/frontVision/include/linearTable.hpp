 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * linearTable.hpp
 *
 *  Created on: May 4, 2016
 *      Author: Tim Kouters
 *  Copied from: http://codereview.stackexchange.com/questions/21407/standard-library-like-linear-interpolation-table
 */

#ifndef LINEARTABLE_HPP_
#define LINEARTABLE_HPP_


#include <functional>
#include <utility>
#include <memory>

#include "lookupDetail.hpp"

namespace lookup
{

template <typename Key,
          typename Value,
          typename Compare = std::less<Key>,
          typename Allocator = std::allocator<std::pair<const Key, Value> >
         >
class unbounded_linear_table
  : public detail::basic_lookup_table<Key, Value, Compare, Allocator>
{

private:

    typedef detail::basic_lookup_table<Key, Value, Compare, Allocator> base;

public:

    typedef typename base::iterator        iterator;
    typedef typename base::const_iterator  const_iterator;
    typedef typename base::size_type       size_type;
    typedef typename base::allocator       allocator;
    typedef typename base::key_type        key_type;
    typedef typename base::mapped_type     mapped_type;
    typedef typename base::value_type      value_type;
    typedef typename base::key_compare     key_compare;
    typedef typename base::reference       reference;
    typedef typename base::const_reference const_reference;
    typedef typename base::pointer         pointer;
    typedef typename base::const_pointer   const_pointer;


    //Returns an unbounded linear interpolation based on key.
    //Unbounded -> if the key is less than the minimum key in
    //the map, it will return the minimum value, if it is greater
    //than the maximum, it will return the maximum.

    mapped_type linear_interp(const Key& k) const
    {
        //First, test to see if the exact key
        //is actually in the table.
        const_iterator find = base::find(k);

        if(find != base::end()) {
            return find->second;
        }

        const_iterator higher = base::upper_bound(k);

        //Lower constraint; upper_bound is less than the
        //min table value
        if(higher == base::begin()) {
            return higher->second;
        }

        //Higher constraint check; upper bound (may)
        //be greater than max table value.
        if(higher == base::end()) {
            const_iterator end_iter = base::end();
            --end_iter;
            if(base::cmp_(end_iter->first, k))
                return end_iter->second;
        }

        const_iterator lower = higher;
        --lower;

        key_type diff_low = k - lower->first;
        key_type total = higher->first - lower->first;

        //Linearlly interpolate between lower and higher values
        return lower->second + (diff_low / total) *
               (higher->second - lower->second);
    }

}; //end class unbounded_linear_table

typedef unbounded_linear_table<double, double> unbounded_lookup1d;

} //end namespace lookup

#endif /* LINEARTABLE_HPP_ */
