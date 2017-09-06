 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * lookupDetail.hpp
 *
 *  Created on: May 4, 2016
 *      Author: Tim Kouters
 *  Copied from: http://codereview.stackexchange.com/questions/21407/standard-library-like-linear-interpolation-table
 */

#ifndef LOOKUPDETAIL_HPP_
#define LOOKUPDETAIL_HPP_

#include <map>

namespace lookup
{
namespace detail
{

template <typename Key,
          typename Value,
          typename Compare,
          typename Allocator>
class basic_lookup_table
{
private:

    typedef std::map<Key, Value, Compare, Allocator> container;
    container   table_;

public:

    typedef typename container::iterator             iterator;
    typedef typename container::const_iterator       const_iterator;
    typedef typename container::size_type            size_type;
    typedef typename container::reference            reference;
    typedef typename container::const_reference      const_reference;
    typedef typename container::pointer              pointer;
    typedef typename container::const_pointer        const_pointer;
    typedef typename container::value_type           value_type;
    typedef Allocator                                allocator;
    typedef Key                                      key_type;
    typedef Value                                    mapped_type;
    typedef Compare                                  key_compare;

protected:

    key_compare cmp_;

    //Disallow polymorphic usage through derived pointer
    ~basic_lookup_table()
    { }

    iterator upper_bound(const Key& k)
    {
        return table_.upper_bound(k);
    }

    const_iterator upper_bound(const Key& k) const
    {
        return table_.upper_bound(k);
    }

    iterator lower_bound(const Key& k)
    {
        return table_.lower_bound(k);
    }

    const_iterator lower_bound(const Key& k) const
    {
        return table_.lower_bound(k);
    }

    iterator find(const Key& k)
    {
        return table_.find(k);
    }

    const_iterator find(const Key& k) const
    {
        return table_.find(k);
    }

public:

    void insert(const key_type& key, const mapped_type& value)
    {
        table_.insert(std::make_pair(key, value));
    }

#if __cplusplus >= 201103L

    void insert(key_type&& key, mapped_type&& value)
    {
        table_.insert(std::make_pair(key, value));
    }

#endif

    bool erase_key(const key_type& k)
    {
        size_type s = table_.erase(k);
        return s != 0;
    }

    void erase_greater(const key_type& k)
    {
        iterator bound = table_.upper_bound(k);
        table_.erase(bound, table_.end());
    }

    void erase_less(const key_type& k)
    {
        iterator bound = table_.lower_bound(k);
        table_.erase(table_.begin(), bound);
    }

    void clear()
    {
        table_.clear();
    }

    iterator begin()
    {
        return table_.begin();
    }

    const_iterator begin() const
    {
        return table_.begin();
    }

    iterator end()
    {
        return table_.end();
    }

    const_iterator end() const
    {
        return table_.end();
    }
};

} //end namespace detail
} //end namespace lookup

#endif /* LOOKUPDETAIL_HPP_ */
