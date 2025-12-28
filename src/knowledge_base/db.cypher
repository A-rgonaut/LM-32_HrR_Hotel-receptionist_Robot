MATCH (n) DETACH DELETE n;

CREATE

(o:Ospite {nome: "Peppe", cognome: "Rossi", eta: 30, lingua: "IT", bpm_attuale: 80, pressione_max_attuale: 120, pressione_min_attuale: 80}),

(l1:Hall {x: 0.0, y: 0.0}),
(l2:Stanza {x: 15.0, y: 5.0, nome: "Camera 1"}),
(l3:CoffeeRoom {x: 25.0, y: 10.0}),

(p1:Prenotazione {data_inizio: datetime("2026-01-12T15:00:00"), data_fine: datetime("2026-01-16T12:00:00")}),

(o)-[:EFFETTUA]->(p1),
(p1)-[:ASSOCIATA_A]->(l2),

(r1:Robot {nome: "Pippor"}),

(pat1:Cardiopatia {bpm_anomala: 100, bpm_allerta: 150}),

(o)-[:SOFFRE_DI]->(pat1),

(i1:Citta {nome: "vino"}),
(i2:Montagna {nome: "trekking"}),

(o)-[:HA_INTERESSE]->(i1),
(o)-[:HA_INTERESSE]->(i2),

(e1:EventoCitta {nome: "Degustazione Cantina Cellaro", data_ora: datetime("2026-01-14T21:00:00")}),
(e2:EventoMontagna {nome: "Escursione Monte Genuardo", data_ora: datetime("2026-01-14T08:00:00")}),
(e3:EventoMare {nome: "Giro in barca Porto Palo", data_ora: datetime("2026-01-14T12:00:00")}),

(sug1:Suggerimento {data_ora: datetime(), idoneo: true}),
(sug2:Suggerimento {data_ora: datetime(), idoneo: true}),
(sug3:Suggerimento {data_ora: datetime(), idoneo: true}),

(r1)-[:GENERA]->(sug1),
(r1)-[:GENERA]->(sug2),
(r1)-[:GENERA]->(sug3),

(sug1)-[:RIFERITO_A]->(e1),
(sug2)-[:RIFERITO_A]->(e2),
(sug3)-[:RIFERITO_A]->(e3),

(sug1)-[:INVIATO_A]->(o),
(sug2)-[:INVIATO_A]->(o),
(sug3)-[:INVIATO_A]->(o),

(s1:Specialista {nome: "Giuseppe", cognome: "Verdi", specialita: "elettricista", numero_telefono: "333 3333 333"}),
(s2:Specialista {nome: "Francesco", cognome: "Bianchi", specialita: "idraulico", numero_telefono: "333 4433 333"}),
(s3:Specialista {nome: "Giulio", cognome: "Neri", specialita: "medico", numero_telefono: "333 5533 333"}),

(m0:PrevisioneMeteo {data_ora: datetime("2026-01-14T00:00:00"), condizione: "sereno", gradi: 10.0}),
(m1:PrevisioneMeteo {data_ora: datetime("2026-01-14T01:00:00"), condizione: "sereno", gradi: 9.9}),
(m2:PrevisioneMeteo {data_ora: datetime("2026-01-14T02:00:00"), condizione: "sereno", gradi: 9.9}),
(m3:PrevisioneMeteo {data_ora: datetime("2026-01-14T03:00:00"), condizione: "sereno", gradi: 9.8}),
(m4:PrevisioneMeteo {data_ora: datetime("2026-01-14T04:00:00"), condizione: "sereno", gradi: 9.7}),
(m5:PrevisioneMeteo {data_ora: datetime("2026-01-14T05:00:00"), condizione: "sereno", gradi: 9.7}),
(m6:PrevisioneMeteo {data_ora: datetime("2026-01-14T06:00:00"), condizione: "sereno", gradi: 10.2}),
(m7:PrevisioneMeteo {data_ora: datetime("2026-01-14T07:00:00"), condizione: "sereno", gradi: 12.0}),
(m8:PrevisioneMeteo {data_ora: datetime("2026-01-14T08:00:00"), condizione: "sereno", gradi: 13.0}),
(m9:PrevisioneMeteo {data_ora: datetime("2026-01-14T09:00:00"), condizione: "nuvoloso", gradi: 11.0}),
(m10:PrevisioneMeteo {data_ora: datetime("2026-01-14T10:00:00"), condizione: "nuvoloso", gradi: 10.8}),
(m11:PrevisioneMeteo {data_ora: datetime("2026-01-14T11:00:00"), condizione: "pioggia", gradi: 10.0}),
(m12:PrevisioneMeteo {data_ora: datetime("2026-01-14T12:00:00"), condizione: "pioggia", gradi: 10.0}),
(m13:PrevisioneMeteo {data_ora: datetime("2026-01-14T13:00:00"), condizione: "pioggia", gradi: 9.9}),
(m14:PrevisioneMeteo {data_ora: datetime("2026-01-14T14:00:00"), condizione: "pioggia", gradi: 9.7}),
(m15:PrevisioneMeteo {data_ora: datetime("2026-01-14T15:00:00"), condizione: "sereno", gradi: 10.0}),
(m16:PrevisioneMeteo {data_ora: datetime("2026-01-14T16:00:00"), condizione: "sereno", gradi: 10.1}),
(m17:PrevisioneMeteo {data_ora: datetime("2026-01-14T17:00:00"), condizione: "sereno", gradi: 10.1}),
(m18:PrevisioneMeteo {data_ora: datetime("2026-01-14T18:00:00"), condizione: "sereno", gradi: 10.0}),
(m19:PrevisioneMeteo {data_ora: datetime("2026-01-14T19:00:00"), condizione: "sereno", gradi: 10.0}),
(m20:PrevisioneMeteo {data_ora: datetime("2026-01-14T20:00:00"), condizione: "sereno", gradi: 10.0}),
(m21:PrevisioneMeteo {data_ora: datetime("2026-01-14T21:00:00"), condizione: "sereno", gradi: 9.9}),
(m22:PrevisioneMeteo {data_ora: datetime("2026-01-14T22:00:00"), condizione: "sereno", gradi: 9.9}),
(m23:PrevisioneMeteo {data_ora: datetime("2026-01-14T23:00:00"), condizione: "sereno", gradi: 10.0}),

(g1:Condizionatore {
    tipo: "elettrico",
    data_ora: datetime(),
    temperatura_stanza: 17,
    temperatura_impostata: 26,
    modalita: "HEAT"
}),
(g2:Lavandino {
    tipo: "idraulico",
    data_ora: datetime(),
    perdita: true
}),
(g3:Phon {
    tipo: "elettrico",
    data_ora: datetime(),
    principio_incendio: true
}),

(o)-[:SEGNALA]->(g1),
(o)-[:SEGNALA]->(g2),
(o)-[:SEGNALA]->(g3),

(r1)-[:NOTIFICA]->(s1),
(r1)-[:NOTIFICA]->(s2),
(r1)-[:NOTIFICA]->(s1),

(es1:BattitiElevati {
    data_ora: datetime(),
    bpm: 158
}),
(es2:PressioneElevata {
    data_ora: datetime(),
    pressione_min: 131,
    pressione_max: 153
}),

(r1)-[:RILEVA]->(es1),
(r1)-[:NOTIFICA]->(s3);
