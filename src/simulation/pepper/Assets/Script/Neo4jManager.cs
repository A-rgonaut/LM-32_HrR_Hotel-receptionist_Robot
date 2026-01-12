using Neo4j.Driver;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Threading.Tasks;
using UnityEngine;

public class Neo4jManager : MonoBehaviour
{
    private string _uri;
    private string _username;
    private string _password;

    private IDriver _driver;

    public struct PersonData
    {
        public int id;
        public string nome;
        public string cognome;
    }

    void Awake()
    {
        // 1. Carichiamo le credenziali PRIMA di connetterci
        LoadCredentialsFromEnv();

        // 2. Controllo sicurezza
        if (string.IsNullOrEmpty(_uri) || string.IsNullOrEmpty(_password))
        {
            Debug.LogError("ERRORE CRITICO: Credenziali Neo4j mancanti! Controlla il file .env.");
            return; // Blocchiamo tutto per evitare crash del driver
        }

        // 3. Configurazione Driver
        _driver = GraphDatabase.Driver(_uri, AuthTokens.Basic(_username, _password));
    }

    private void LoadCredentialsFromEnv()
    {
        // Percorso universale per la cartella StreamingAssets
        string envPath = "../ros2_ws/.env";

        if (File.Exists(envPath))
        {
            string[] lines = File.ReadAllLines(envPath);

            foreach (string line in lines)
            {
                // Ignora commenti (#) o righe vuote
                if (string.IsNullOrWhiteSpace(line) || line.Trim().StartsWith("#")) continue;

                // Divide la riga al primo '=' trovato
                string[] parts = line.Split(new char[] { '=' }, 2);

                if (parts.Length != 2) continue;

                string key = parts[0].Trim();
                string value = parts[1].Trim().Replace("\"", "");

                switch (key)
                {
                    case "NEO4J_URI":
                        _uri = value;
                        break;
                    case "NEO4J_USER":
                        _username = value;
                        break;
                    case "NEO4J_PASS":
                        _password = value;
                        break;
                }
            }

            Debug.Log("Credenziali caricate da .env con successo.");
        }
        else
        {
            Debug.LogError($"File .env non trovato nel percorso: {envPath}");
        }
    }

    void OnApplicationQuit()
    {
        DisposeDriver();
    }

    void OnDestroy()
    {
        DisposeDriver();
    }

    private void DisposeDriver()
    {
        if (_driver != null)
        {
            _driver.Dispose();
            _driver = null;
        }
    }

    public async Task<List<PersonData>> GetCharactersFromDB()
    {
        List<PersonData> results = new List<PersonData>();

        if (_driver == null) return results;

        try
        {
            await _driver.VerifyConnectivityAsync();

            await using var session = _driver.AsyncSession();
            var query = "MATCH(n: Ospite) RETURN " +
                        "id(n) AS id, " +
                        "n.nome_ospite AS nome, " +
                        "n.cognome_ospite AS cognome" +
                        "ORDER BY id ASC";

            await session.ExecuteReadAsync(async tx =>
            {
                var cursor = await tx.RunAsync(query);
                while (await cursor.FetchAsync())
                {
                    var record = cursor.Current;
                    PersonData p = new PersonData
                    {
                        id = Convert.ToInt32(record["id"]),
                        nome = record["nome"].As<string>(),
                        cognome = record["cognome"].As<string>()
                    };
                    results.Add(p);
                }
            });
        }
        catch (Exception ex)
        {
            Debug.LogError($"[Neo4j Error]: {ex.Message}");
        }

        return results;
    }
}
