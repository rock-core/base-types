#ifndef BASE_NAMED_VECTOR_HPP
#define BASE_NAMED_VECTOR_HPP

#include <stdexcept>
#include <vector>
#include <algorithm>
#include <string>

namespace base
{
    template <class T> 
    struct NamedVector
    {
	/** Exception thrown when trying to find the index of an element by
	 * name, but the name does not exist
	 */
	struct InvalidName : public std::runtime_error
	{
	    std::string name;
	    InvalidName(std::string const& name)
		: std::runtime_error("trying to access element " + name + 
			", but there is no element with that name on this structure")
		  , name(name) {}

	    ~InvalidName() throw() {}
	};

	/** The names of the elements described in this structure, in the same
	 * order than the 'elements' field below
	 */
	std::vector<std::string> names;

	/** The element vector */
	std::vector<T> elements;

        /** Returns true if the joint vector is properly filled */
        bool hasNames() const
        {
            return !names.empty() && !names[0].empty();
        }

	/** Returns the state information for the given element */
	const T& getElementByName(std::string name) const
	{
	    return elements.at( mapNameToIndex( name ) );
	}

	/** convenience method, does the same as the getElementByName */
	const T& operator[](std::string name) const
	{
	    return elements.at( mapNameToIndex( name ) );
	}

	/** convenience method, does the same as the getElementByName */
	T& operator[](std::string name)
	{
	    return elements.at( mapNameToIndex( name ) );
	}

	/** returns the element with the given index */
	const T& operator[]( size_t index ) const
	{
	    return elements.at( index );
	}

	/** returns the element with the given index */
	T& operator[]( size_t index )
	{
	    return elements.at( index );
	}

	/** Resize the elements vector to this size
	*/
	void resize(size_t size)
	{
	    elements.resize(size);
	    names.resize(size);
	}

	/** Returns the number of elements reported in this structure
	*/
	size_t size() const
	{
	    return elements.size();
	}

        /** Returns true if this does not contain any elements
         */
        bool empty() const
        {
            return elements.empty();
        }

	/** Clears the contents of elements and names vector
	*/
	void clear()
	{
	    elements.clear();
	    names.clear();
	}

	/** Returns the index that corresponds to the given name
	 *
	 * @throws InvalidName if the given name does not exist on this array
	 */
	size_t mapNameToIndex(std::string const& name) const
	{
	    std::vector<std::string>::const_iterator it = find(names.begin(), names.end(), name);
	    if (it == names.end())
		throw InvalidName(name);
	    return it - names.begin();
	}
    };
}

#endif

