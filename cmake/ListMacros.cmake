# Some Macros to use with lists

# Get the first element of a list
macro(first_element VAR)
    set(${VAR} ${ARGV1})
endmacro()
# Get element with index INDEX of a list (empty variable if index is out of range)
macro(element_of VAR INDEX)
    set(${VAR} ${ARGV${INDEX}})
endmacro()

# Split list into lists by keys
# usage: list_keys_split( ${PREFIX_NAME} ${KEYS} ${LIST} )
# Splits ${LIST} into lists sorted by occurencs of a key from ${KEYS}.
# Defines
#   foreach(KEY ${KEYS})
#       ${PREFIX_NAME}_${KEY}   : List with the elements occuring after KEY
#       ${PREFIX_NAME}_${KEY}_FOUND : ON if the respective KEY is found in the list.
#   end 
# Example:
#   set(MyList SOURCES test1 test2 DEPS hallo1 NOINSTALL
#   set(Keys SOURCES DEPS NOINSTALL TESTS)
#   list_keys_split(MyLists Keys MyList)
# Result:
#   MyLists_SOURCES_FOUND ON
#   MyLists_DEPS_FOUND ON
#   MyLists_NOINSTALL ON
#   MyLists_TESTS OFF
#   MyLists_SOURCES test1;test2
#   MyLists_DEPS hallo1
#   MyLists_NOINSTALL
macro(list_keys_split LISTNAME KEYS)
    first_element(KEY1 ${KEYS})
    set(KEYLIST ${KEYS}) 
    set(${LISTNAME}_MODE "${KEY1}")
    foreach(KEY ${KEYLIST})
        set(${LISTNAME}_${KEY}_FOUND OFF)
    endforeach()
    foreach(ELEMENT ${ARGN})
        list(FIND KEYLIST ${ELEMENT} IS_KEY)
        if(IS_KEY GREATER -1)
            set(${LISTNAME}_MODE "${ELEMENT}")
            set(${LISTNAME}_${${LISTNAME}_MODE}_FOUND ON)
        else()
            list(APPEND ${LISTNAME}_${${LISTNAME}_MODE} "${ELEMENT}")
        endif()
    endforeach()
endmacro()
