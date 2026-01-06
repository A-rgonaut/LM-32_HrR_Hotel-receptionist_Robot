using UnityEngine;
using System.IO;

public class MapExporter : MonoBehaviour
{
    public Camera mapCamera; // Trascina qui la tua Orthographic Camera
    public int resolutionX = 1024;
    public int resolutionY = 1024;
    public string fileName = "map_ros";

    [ContextMenu("Cattura Mappa 2D")]
    public void CaptureMap()
    {
        if (mapCamera == null)
        {
            mapCamera = GetComponent<Camera>();
        }

        // 1. Configura la RenderTexture (la "pellicola" della foto)
        RenderTexture rt = new RenderTexture(resolutionX, resolutionY, 24);
        mapCamera.targetTexture = rt;

        // 2. Scatta la foto
        Texture2D screenShot = new Texture2D(resolutionX, resolutionY, TextureFormat.RGB24, false);
        mapCamera.Render();

        // 3. Leggi i pixel
        RenderTexture.active = rt;
        screenShot.ReadPixels(new Rect(0, 0, resolutionX, resolutionY), 0, 0);
        mapCamera.targetTexture = null;
        RenderTexture.active = null;
        DestroyImmediate(rt); // Pulizia

        // 4. Converti in bytes (PNG)
        byte[] bytes = screenShot.EncodeToPNG();

        // 5. Salva il file
        string path = Path.Combine(Application.dataPath, fileName + ".png");
        File.WriteAllBytes(path, bytes);

        Debug.Log($"Mappa salvata in: {path}");
        Debug.Log($"IMPORTANTE: Aggiorna il file YAML con resolution: {CalculateResolution()}");
    }

    // Calcola la risoluzione in metri/pixel per ROS
    private float CalculateResolution()
    {
        // La Size ortografica è metà dell'altezza verticale in metri
        float verticalHeightMeters = mapCamera.orthographicSize * 2;
        return verticalHeightMeters / resolutionY;
    }
}