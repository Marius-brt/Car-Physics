using System;
using System.Collections.Generic;

namespace Physics
{
    struct Tire
    {
        /// <summary>
        /// Wheel static radius [m]
        /// </summary>
        public float rws;
        /// <summary>
        /// Wheel dynamic radius [m]
        /// </summary>
        public float rwd;
        /// <summary>
        /// Tire friction coeff
        /// </summary>
        public float miua;
        /// <summary>
        /// Rear axle load coeff
        /// </summary>
        public float load;
    }

    struct Gearbox
    {
        /// <summary>
        /// Lowest gear
        /// </summary>
        public int gearMin;
        /// <summary>
        /// Highest gear
        /// </summary>
        public int gearMax;
        /// <summary>
        /// Gears ratio
        /// </summary>
        public float[] gearRat;
        /// <summary>
        /// Final drive ratio (differential)
        /// </summary>
        public float i0;
        /// <summary>
        /// Driveline efficiency
        /// </summary>
        public float eff;
    }

    struct Vehicle
    {
        /// <summary>
        /// Vehicle mass [kg]
        /// </summary>
        public int mass_curb;
        /// <summary>
        /// Driver mass [kg]
        /// </summary>
        public int mass_driver;
        /// <summary>
        /// Total mass [kg]
        /// </summary>
        public int mass;
        /// <summary>
        /// Drag coefficient
        /// </summary>
        public float cd;
        /// <summary>
        /// Frontal area [m²]
        /// </summary>
        public float fa;
        /// <summary>
        /// Air density [kg/m3]
        /// </summary>
        public float ro;
    }

    struct Road
    {
        /// <summary>
        /// Road slope angle [rad]
        /// </summary>
        public float slope;
        /// <summary>
        /// Road load coeff
        /// </summary>
        public float cr;
    }

    struct Engine
    {
        /// <summary>
        /// Engine speed for maximum torque [rpm]
        /// </summary>
        public int NmaxTq;
        /// <summary>
        /// Engine speed for maximum power [rpm]
        /// </summary>
        public int NmaxPwr;
        /// <summary>
        /// Engine torque curve at full load [Nm]
        /// </summary>
        public float[] tqFullLoad;
        /// <summary>
        /// Engine speed axis [rpm]
        /// </summary>
        public float[] NtqFullLoad;
        /// <summary>
        /// Maximum engine speed [rpm]
        /// </summary>
        public int Nmax;
        /// <summary>
        /// Minimum engine speed [rpm]
        /// </summary>
        public int Nmin;
        /// <summary>
        /// Engine power curve at full load [HP]
        /// </summary>
        public List<float> pwrFullLoad;
    }

    class Car
    {
        public float g = 9.81f;
        public float simulationTime = 60f;
        public float dt = 0.01f;
        public float time = 0;

        public Gearbox gbx = new Gearbox { eff = 0.85f, gearMax = 8, gearMin = 1, gearRat = new float[] { 4.71f, 3.14f, 2.11f, 1.67f, 1.29f, 1.00f, 0.84f, 0.67f }, i0 = 3.31f };
        public Tire tire = new Tire { rws = 0.343f, load = 0.65f, miua = 1.1f };
        public Vehicle veh = new Vehicle { mass_curb = 1741, mass_driver = 80, cd = 0.36f, fa = 2.42f, ro = 1.202f };
        public Road road = new Road { cr = 0.011f, slope = 0 };
        public Engine eng = new Engine { NmaxPwr = 6000, NmaxTq = 3500, Nmax = 6500, Nmin = 1000, NtqFullLoad = new float[]{ 1000, 2020, 2990, 3500, 5000, 6500 }, tqFullLoad = new float[] { 306, 385, 439, 450, 450, 367 }, pwrFullLoad = new List<float>() };

        public float Eng_spd;
        public int GearIndex;
        public float sVehAcc_mps2 = 0;
        public float sWlhTotTrc_N = 0;
        public float sWlhFricLim_N = 0;
        public float sWlhRolRes_N = 0;
        public float sVehSpd_ms = 0;
        public float sVehSpd_kph = 0;
        public float sTrnTq_Nm = 0;
        public float sEngSpd_rpm = 0;
        public float sEngTq_Nm = 0;
        public float sEngPwd_Hp = 0;
        public float sTrnSpd_rads = 0;

        public Car()
        {
            Eng_spd = eng.Nmin;
            GearIndex = 1;
            float[] mult = MatrixMultiplication(eng.tqFullLoad, eng.NtqFullLoad);
            for (int i = 0; i < mult.Length; i++)
                eng.pwrFullLoad.Add((float)((1.36f / 1000) * (Math.PI / 30) * mult[i]));
            tire.rwd = 0.98f * tire.rws;
            veh.mass = veh.mass_curb + veh.mass_driver;
            while(time < simulationTime)
            {
                float Eng_tq = CalcEngine(Eng_spd);
                (float torque, float speed) = CalcTransmission(Eng_tq, sTrnSpd_rads);
                float spd = CalcVehicle(torque);
                Console.WriteLine(sVehSpd_kph);
                time += dt;
            }
        }

        private float[] MatrixMultiplication(float[] A, float[] B)
        {
            float[] C = new float[A.Length];
            for (int i = 0; i < A.Length; i++)
                C[i] = A[i] * B[i];
            return C;
        }

        private float Interpolate(float x, float[] X, float[] Y)
        {
            if (X.Length != Y.Length) throw new ArgumentException("Arrays must be of equal length.");
            int i = Array.FindIndex(X, k => x <= k);
            if (i <= 0)
                return Y[0];
            else if (i > Y.Length - 1)
                return Y[Y.Length - 1];
            else
                return Y[i - 1] + (x - X[i - 1]) * (Y[i] - Y[i - 1]) / (X[i] - X[i - 1]);
        }

        private float CalcVehicle(float Whl_tq)
        {
            float Fl = veh.mass * g * tire.miua * tire.load;
            float Fs = (float)(veh.mass * g * Math.Sin(road.slope));
            float Fr = (float)(veh.mass * g * road.cr * Math.Cos(road.slope));
            float Tf = Whl_tq / tire.rwd;
            float min = Math.Min(Fl, Tf);
            float Fa = 0.5f * veh.ro * veh.cd * veh.fa * sVehSpd_ms * sVehSpd_ms;
            float Fb = (min + Fs + Fr /*+ Fa*/) / veh.mass;
            //Console.WriteLine($"Min: {min}, Fs: {Fs}, Fr: {Fr}, Fa: {Fa}");
            float sp = sVehSpd_ms + dt * Fb;

            sVehAcc_mps2 = Fb;
            sVehSpd_ms = sp;
            sVehSpd_kph = sp * 3.6f;
            sWlhTotTrc_N = Tf;
            sWlhFricLim_N = Fl;
            sWlhRolRes_N = Fs + Fr + Fa;
            sTrnSpd_rads = sp / tire.rwd;

            return sp / tire.rwd;
        }

        private float CalcEngine(float Eng_spd)
        {
            float MIF = Eng_spd / (1 + 0.1f * Eng_spd);
            //Console.WriteLine(MIF);
            if (MIF < eng.Nmin) MIF = eng.Nmin;
            if (MIF > eng.Nmax) MIF = eng.Nmax;
            float interp = Interpolate(MIF, eng.NtqFullLoad, eng.tqFullLoad);
           // Console.WriteLine(interp);

            sEngPwd_Hp = (float)((Math.PI / 30) * MIF * interp * 0.001f * 1.36f);
            sEngTq_Nm = interp;
            sEngSpd_rpm = MIF;

            return interp;
        }

        private (float, float) CalcTransmission(float Eng_tq, float Trn_spd)
        {
            float ratio = gbx.gearRat[CalcGear(Eng_spd) - 1];
            float spd = (float)(Trn_spd * (30 / Math.PI) * gbx.i0 * ratio);
            float torque = gbx.eff * Eng_tq * ratio * gbx.i0;

            Eng_spd = spd;
            sTrnTq_Nm = torque;

            return (torque, spd);
        }

        private int CalcGear(float EngineSpeed)
        {
            if(EngineSpeed > eng.NmaxPwr && GearIndex < gbx.gearMax)
                GearIndex++;
            else if(EngineSpeed < eng.NmaxTq && GearIndex > gbx.gearMin)
                GearIndex--;
            return GearIndex;
        }
    }
}
