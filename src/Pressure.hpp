#ifndef __BASE_PRESSURE_HPP__
#define __BASE_PRESSURE_HPP__

namespace base
{
    /** Representation of a pressure value
     *
     * It is internally normalized to a pressure in pascals (the SI unit for
     * pressure), but provides accessors and initializers to convert to the most
     * common pressure units (bar and PSI)
     */
    class Pressure
    {
    public:
        /** Pressure in pascals
         *
         * This is made public so that it can be used directly in Rock
         * components. You usually should not access it, but instead use one of
         * the initializers / accessors
         */
        float pascal;

        /** 
         * default constructor, which will initialize the value to unknown (NaN)
         */
        Pressure();

    public:
        /** Create a pressure object using a value in pascals */
        static Pressure fromPascal(float pascal);

        /** Create a pressure object using a value in bars */
        static Pressure fromBar(float bar);

        /** Create a pressure object using a value in PSI */
        static Pressure fromPSI(float psi);

        /** Returns the raw pressure value in pascals */
        float toPa() const;

        /** Returns the raw pressure value in bar */
        float toBar() const;

        /** Returns the raw pressure value in psi */
        float toPSI() const;
    };
}

#endif

