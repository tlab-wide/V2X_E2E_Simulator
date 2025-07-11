using System;
using System.Linq;
using AWSIM;
using UnityEngine;
using GeographicLib; // Add this to use GeographicLib

public class UTMConvertor : MonoBehaviour
{
    [SerializeField] private Transform m_Target;

    [SerializeField] private string Z_utm = "54S";

    [SerializeField] private double E_utm = 404542.530;

    [SerializeField] private double N_utm = 3972957.923;

    [SerializeField] private double lat_pivot = 35.896251;

    [SerializeField] private double lon_pivot = 139.942266;

    private Vector3 utm_vector;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        // todo section   
        // Zone is Z_utm (string), Easting is E_utm (float), Northing is N_utm (float)

        // GeographicLib.UTMUPS.ToLatLon(E_utm, N_utm, Z_utm, out lat, out lon);
        //
        // Debug.Log("Latitude: " + lat + " Longitude: " + lon);
    }

    // Update is called once per frame
    void Update()
    {
        // Vector3 pos_utm = ROS2Utility.UnityToRosPosition(m_Target.transform.position) + utm_vector;

        (double lat, double lon) = GeographicLib.UTMUPS.Reverse(54, true, m_Target.transform.position.z + E_utm,
            -m_Target.transform.position.x + N_utm);
        Debug.Log($"lat long {lat} + {lon}");
        
    }


    public static Vector2 ToLatLon(double utmX, double utmY, string utmZone)
    {
        double latitude, longitude;
        bool isNorthHemisphere = utmZone.Last() >= 'N';

        var diflat = -0.00066286966871111111111111111111111111;
        var diflon = -0.0003868060578;

        var zone = int.Parse(utmZone.Remove(utmZone.Length - 1));
        var c_sa = 6378137.000000;
        var c_sb = 6356752.314245;
        var e2 = Math.Pow((Math.Pow(c_sa, 2) - Math.Pow(c_sb, 2)), 0.5) / c_sb;
        var e2cuadrada = Math.Pow(e2, 2);
        var c = Math.Pow(c_sa, 2) / c_sb;
        var x = utmX - 500000;
        var y = isNorthHemisphere ? utmY : utmY - 10000000;

        var s = ((zone * 6.0) - 183.0);
        var lat = y / (c_sa * 0.9996);
        var v = (c / Math.Pow(1 + (e2cuadrada * Math.Pow(Math.Cos(lat), 2)), 0.5)) * 0.9996;
        var a = x / v;
        var a1 = Math.Sin(2 * lat);
        var a2 = a1 * Math.Pow((Math.Cos(lat)), 2);
        var j2 = lat + (a1 / 2.0);
        var j4 = ((3 * j2) + a2) / 4.0;
        var j6 = ((5 * j4) + Math.Pow(a2 * (Math.Cos(lat)), 2)) / 3.0;
        var alfa = (3.0 / 4.0) * e2cuadrada;
        var beta = (5.0 / 3.0) * Math.Pow(alfa, 2);
        var gama = (35.0 / 27.0) * Math.Pow(alfa, 3);
        var bm = 0.9996 * c * (lat - alfa * j2 + beta * j4 - gama * j6);
        var b = (y - bm) / v;
        var epsi = ((e2cuadrada * Math.Pow(a, 2)) / 2.0) * Math.Pow((Math.Cos(lat)), 2);
        var eps = a * (1 - (epsi / 3.0));
        var nab = (b * (1 - epsi)) + lat;
        var senoheps = (Math.Exp(eps) - Math.Exp(-eps)) / 2.0;
        var delt = Math.Atan(senoheps / (Math.Cos(nab)));
        var tao = Math.Atan(Math.Cos(delt) * Math.Tan(nab));

        longitude = ((delt * (180.0 / Math.PI)) + s) + diflon;
        latitude = ((lat +
                     (1 + e2cuadrada * Math.Pow(Math.Cos(lat), 2) -
                      (3.0 / 2.0) * e2cuadrada * Math.Sin(lat) * Math.Cos(lat) * (tao - lat)) * (tao - lat)) *
                    (180.0 / Math.PI)) + diflat;

        return new Vector2((float)latitude, (float)longitude);
    }
}