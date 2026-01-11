using UnityEngine;
using Neo4j.Driver; // Importante
using System.Threading.Tasks;
using System.Collections.Generic;
using System;

public class Neo4jManager : MonoBehaviour
{
    // Configurazioni DB
    [Header("Neo4j Settings")]
    public string uri = "bolt://localhost:7687";
    public string username = "neo4j";
    public string password = "pepper123"; 

    private IDriver _driver;

    // Struttura dati temporanea per passare i dati al Selector
    public struct PersonData
    {
        public int id;
        public string nome;
        public string cognome;
    }

    void Awake()
    {
        // Inizializza il driver
        _driver = GraphDatabase.Driver(uri, AuthTokens.Basic(username, password));
    }

    void OnDestroy()
    {
        // Chiudi la connessione quando chiudi il gioco
        _driver?.Dispose();
    }

    // Funzione Asincrona per recuperare i dati
    public async Task<List<PersonData>> GetCharactersFromDB()
    {
        List<PersonData> results = new List<PersonData>();

        try
        {
            // Apriamo una sessione asincrona
            await using var session = _driver.AsyncSession();

            // Eseguiamo la query
            // ORDER BY n.id assicura che arrivino nell'ordine giusto per l'array di Unity
            var query = "MATCH (n:Ospite) RETURN n.id AS id, n.nome_ospite AS nome, n.cognome_ospite AS cognome ORDER BY n.id ASC";

            await session.ExecuteReadAsync(async tx =>
            {
                var cursor = await tx.RunAsync(query);

                // Leggiamo i record uno per uno
                while (await cursor.FetchAsync())
                {
                    var record = cursor.Current;

                    PersonData p = new PersonData();
                    // Neo4j restituisce numeri come Long, in C# dobbiamo castare
                    p.id = Convert.ToInt32(record["id"]);
                    p.nome = record["nome"].As<string>();
                    p.cognome = record["cognome"].As<string>();

                    results.Add(p);
                }
            });
        }
        catch (Exception ex)
        {
            Debug.LogError($"Errore Neo4j: {ex.Message}");
        }

        return results;
    }
}