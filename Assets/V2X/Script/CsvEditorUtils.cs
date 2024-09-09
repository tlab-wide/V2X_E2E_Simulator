using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

public class CsvEditorUtils
{
    //this function can be used when we need to check if the file does not exist, initial with configured setup
    public static void HandleCsvHeader(string logPathCars, string content)
    {
        if (!string.IsNullOrEmpty(logPathCars) && !File.Exists(logPathCars))
        {
            AppendStringToFile(logPathCars, content);
        }
    }


    public static void AppendStringToFile(string filePath, string content)
    {
        using (StreamWriter writer = new StreamWriter(filePath, true))
        {
            writer.Write(content);
        }
    }
    
    
    private static void AppendStringToFile(string filePath, List<string> contents)
    {
        using (StreamWriter writer = new StreamWriter(filePath, true))
        {
            foreach (var content in contents)
            {
                writer.Write(content);
            }
        }
    }
    
    public static List<List<string>> ReadCsvFile(TextAsset textAsset)
    {
        var result = new List<List<string>>();

        try
        {
            var lines = textAsset.text.Split('\n');
            Debug.Log($"**{lines.Length}");
            Debug.Log($"**{textAsset.text}");
            
            
            foreach (var line in lines)
            {
                var values = new List<string>(line.Split(','));
                result.Add(values);
            }
        }
        catch (Exception ex)
        {
            Debug.LogError($"An error occurred while reading the CSV file: {ex.Message}");
        }

        return result;
    }
    
    

}