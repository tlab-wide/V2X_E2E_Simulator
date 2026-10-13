using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace AWSIM
{
    /// <summary>
    /// Displays speed and gear UI of the active vehicle (several vehicles can be registered, e.g. Bus and Lexus).
    /// </summary>
    public class VehicleInfomationUI : MonoBehaviour
    {
        [SerializeField] public List<Vehicle> vehicles = new List<Vehicle>();
        [SerializeField] Text speedText;
        [SerializeField] Text gearText;

        // Single-vehicle field of older versions. Scenes that still have it set are migrated into 'vehicles'.
        [SerializeField, HideInInspector] Vehicle vehicle;

        void Awake()
        {
            if (vehicle != null)
            {
                AddVehicle(vehicle);
                vehicle = null;
            }
        }

        public void AddVehicle(Vehicle newVehicle)
        {
            if (newVehicle != null && !vehicles.Contains(newVehicle))
                vehicles.Add(newVehicle);
        }

        public Vehicle GetActiveVehicle()
        {
            foreach (Vehicle v in vehicles)
            {
                if (v != null && v.gameObject.activeInHierarchy)
                    return v;
            }
            return null;
        }

        void Update()
        {
            var active = GetActiveVehicle();
            if (active == null)
            {
                speedText.text = "";
                gearText.text = "";
                return;
            }

            speedText.text = "" + Mathf.Floor(active.Speed * 3.6f);
            gearText.text = "" + GetShiftString(active.AutomaticShift);
        }

        static string GetShiftString(Vehicle.Shift shift)
        {
            if (shift == Vehicle.Shift.DRIVE)
                return "D";
            if (shift == Vehicle.Shift.NEUTRAL)
                return "N";
            if (shift == Vehicle.Shift.PARKING)
                return "P";
            return "R";
        }
    }
}
