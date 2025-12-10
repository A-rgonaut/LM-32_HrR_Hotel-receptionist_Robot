MATCH (n) DETACH DELETE n;

CREATE

(p:Persona {nome: "Peppe", cognome: "Rossi", eta: 30, lingua: "IT"}),
(b:Braccialetto {}),

(p)-[:INDOSSA]->(b),

(l1:Luogo {x: 0.0, y: 0.0, nome: "Hall", comune: true}),
(l2:Luogo {x: 15.0, y: 5.0, nome: "Camera 1", comune: false}),
(l3:Luogo {x: 25.0, y: 10.0, nome: "Coffee Room", comune: true}),

(p1:Prenotazione {inizio: datetime("2026-01-12T15:00:00"), fine: datetime("2026-01-16T12:00:00")}),

(p)-[:EFFETTUA]->(p1),
(p1)-[:ASSOCIATA]->(l2),
(l1)-[:CONNESSO]->(l2),

(r1:Robot {nome: "Pippor"}),

(pat1:Patologia {nome: "Cardiopatia", sogliaBPM_Anomala: 100, sogliaBPM_Allerta: 150}),

(p)-[:SOFFRE_DI]->(pat1),

(i1:Interesse {nome: "Vino"}),
(i2:Interesse {nome: "Trekking"}),

(p)-[:HA_INTERESSE]->(i1),
(p)-[:HA_INTERESSE]->(i2),

(e1:EventoLocale {nome: "Degustazione Cantina Cellaro", data_ora: datetime("2026-01-14T21:00:00")}),
(e2:EventoLocale {nome: "Escursione Monte Genuardo", data_ora: datetime("2026-01-14T09:00:00"), meteoRichiesto: "sole"}),
(e3:EventoLocale {nome: "Visita Valle dei Templi", data_ora: datetime("2026-01-14T16:00:00")}),

(sug1:Suggerimento {data_ora: datetime()}),
(sug2:Suggerimento {data_ora: datetime()}),

(r1)-[:GENERA]->(sug1),
(r1)-[:GENERA]->(sug2),
(r1)-[:GENERA]->(sug3),
(sug1)-[:INERENTE]->(e1),
(sug2)-[:INERENTE]->(e2),
(sug3)-[:INERENTE]->(e3),
(sug1)-[:INVIATO_A]->(p),
(sug2)-[:INVIATO_A]->(p),
(sug3)-[:INVIATO_A]->(p),

(s1:Specialista {specializzazione: "elettricista", numTel: "333 3333 333"}),
(s2:Specialista {specializzazione: "idraulico", numTel: "333 4433 333"}),
(s3:Specialista {specializzazione: "medico", numTel: "333 5533 333"}),

(m_mattina:PrevisioneMeteo {data_ora: datetime("2026-01-14T07:00:00"), condizione: "sole"}),
(m_pomeriggio:PrevisioneMeteo {data_ora: datetime("2026-01-14T15:00:00"), condizione: "nuvoloso"}),
(m_sera:PrevisioneMeteo {data_ora: datetime("2026-01-14T23:00:00"), condizione: "pioggia"}),

(e3)-[:PREVEDE]->(m_pomeriggio),
(e2)-[:PREVEDE]->(m_mattina),
(e1)-[:PREVEDE]->(m_sera),

(g1:Guasto {
    oggetto: "condizionatore",
    tipo: "elettrico",
    data_ora: datetime("2026-01-13T10:00:00"),
    tempStanza: 17,
    tempImpostata: 26,
    modalita: "HEAT"
}),
(g2:Guasto {
    oggetto: "lavandino",
    tipo: "idraulica",
    data_ora: datetime("2026-01-13T11:00:00")
}),

(p)-[:SEGNALA]->(g1),
(p)-[:SEGNALA]->(g2),
(g1)-[:RISOLVIBILE_DA]->(s1),
(g2)-[:RISOLVIBILE_DA]->(s2),
(r1)-[:NOTIFICA]->(s1),
(r1)-[:NOTIFICA]->(s2),

(es:EmergenzaSanitaria {
    tipo: "malore",
    data_ora: datetime("2026-01-14T23:00:00"),
    pressione: 90,
    battiti: 160,
    temperatura: 36.5
}),

(b)-[:RILEVA]->(es),
(es)-[:AVVIENE]->(l3),
(es)-[:GESTIBILE_DA]->(s3),
(r1)-[:NOTIFICA]->(s3);
