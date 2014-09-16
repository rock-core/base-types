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

    public:
        /** Create a pressure object using a value in pascals */
        static Pressure fromPascal(float pascal)
        {
            Pressure result;
            result.pascal = pascal;
            return result;
        }

        /** Create a pressure object using a value in bars */
        static Pressure fromBar(float bar)
        {
            return fromPascal(bar * 100000);
        }

        /** Create a pressure object using a value in PSI */
        static Pressure fromPSI(float psi)
        {
            return fromPascal(psi * 6894.75729);
        }

        /** Returns the raw pressure value in pascals */
        float toPa() const
        {
            return pascal;
        }

        /** Returns the raw pressure value in bar */
        float toBar() const
        {
            return pascal / 100000;
        }

        /** Returns the raw pressure value in psi */
        float toPSI() const
        {
            return pascal / 6894.75729;
        }
    };
}

#endif

