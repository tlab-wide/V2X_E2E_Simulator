using UnityEngine;

[DisallowMultipleComponent]
public class ScenarioHumanId : MonoBehaviour
{
    private static uint nextId = 0;
    private string humanId;

    public string HumanId
    {
        get
        {
            return humanId;
        }
    }

    public void AssignNewId()
    {
        humanId = nextId.ToString();
        nextId++;
    }
}
