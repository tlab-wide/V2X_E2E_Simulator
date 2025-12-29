using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace AWSIM
{
    /// <summary>
    /// Displays speed and gear UI.
    /// </summary>
    public class VehicleInfomationUI : MonoBehaviour
    {
        [SerializeField] public List<Vehicle> vehicles;
        [SerializeField] Text speedText;
        [SerializeField] Text gearText;

        public void AddVehicle(Vehicle vehicle)
        {
            vehicles.Add(vehicle);
        }
        public Vehicle GetActiveVehicle()
        {
            foreach (Vehicle vehicle in vehicles)
            {
                if (vehicle.gameObject.activeInHierarchy)
                {
                    return vehicle;
                }
            }
            return null;
        }
        
        void Update()
        {
            foreach (var vehicle in vehicles)
            {
                if(!vehicle.gameObject.activeInHierarchy) continue;
                
                if (!vehicle) {
                    speedText.text = "";
                    gearText.text = "";
                    return;
                }

                speedText.text = "" + Mathf.Floor(vehicle.Speed * 3.6f);
                gearText.text = "" + GetShiftString(vehicle.AutomaticShift);

                static string GetShiftString(Vehicle.Shift shift)
                {
                    string shiftString = "";
                    if (shift == Vehicle.Shift.DRIVE)
                        shiftString = "D";
                    else if (shift == Vehicle.Shift.NEUTRAL)
                        shiftString = "N";
                    else if (shift == Vehicle.Shift.PARKING)
                        shiftString = "P";
                    else
                        shiftString = "R";

                    return shiftString;
                }
            }
            
        }
    }
}