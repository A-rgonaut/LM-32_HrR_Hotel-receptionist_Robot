using UnityEngine;

public class CharacterProfile : MonoBehaviour
{
    [Header("Dati Personaggio")]
    public int id;
    public string nome;
    public string cognome;

    // Questa è la funzione che verrà chiamata dal bottone
    public void AttivaPersonaggio()
    {
        Debug.Log($"Hai selezionato: {nome} {cognome} (ID: {id})");

        // Qui inserisci la logica del tuo evento
        // Esempio: StartGame(id);
    }
}