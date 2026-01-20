package com.example.l;

import java.io.File;
import java.util.Set;

import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.model.*;
import org.semanticweb.owlapi.reasoner.OWLReasoner;
import org.semanticweb.owlapi.reasoner.OWLReasonerFactory;
import org.semanticweb.owlapi.reasoner.SimpleConfiguration;

import openllet.owlapi.OpenlletReasonerFactory;

import com.clarkparsia.owlapi.explanation.DefaultExplanationGenerator;

public class SpiegamiTutto {
    private static boolean DEBUG = false;

    private static void log(String msg) {
        if (DEBUG) {
            System.out.println(msg);
        }
    }

    public static void main(String[] args) {
        if (args.length > 2) {
            DEBUG = Boolean.parseBoolean(args[2]);
        }
        if (DEBUG) {
            System.setProperty("org.slf4j.simpleLogger.defaultLogLevel", "info");
        } else {
            System.setProperty("org.slf4j.simpleLogger.defaultLogLevel", "off");
        }
        if (args.length < 2) {
            log("Uso: java -jar reasoner.jar <OwlFile> <ParentClass> [<DEBUG(true/false)>]");
            System.exit(1);
        }
        String owlFilePath     = args[0];
        String parentClassName = args[1];

        OWLOntologyManager man = OWLManager.createOWLOntologyManager();

        log("--- AVVIO SISTEMA DI RAGIONAMENTO ---");

        File fileOntologia = new File(owlFilePath);

        if (fileOntologia.exists()) {
            log("Il file " + fileOntologia.getName() + " è stato trovato nel percorso: " + fileOntologia.getAbsolutePath());
            try {  // Carica l'ontologia dal file
                OWLOntology ontologia = man.loadOntologyFromOntologyDocument(fileOntologia);
                OWLDataFactory df = man.getOWLDataFactory();
                log("Ontologia caricata con successo!");
                log("-> IRI/Nome dell'Ontologia: " + ontologia.getOntologyID().getOntologyIRI().orElse(IRI.create("Nessun nome esplicito")));
                log("-> Numero totale di Assiomi: " + ontologia.getAxiomCount());
                log("-> Numero di Classi: " + ontologia.getClassesInSignature().size());
                log("-> Numero di Data Properties: " + ontologia.getDataPropertiesInSignature().size());
                log("-> Numero di Object Properties: " + ontologia.getObjectPropertiesInSignature().size());
                log("-> Numero di Individui (Named Individuals): " + ontologia.getIndividualsInSignature().size());
                log("\n### Esempio Classi Trovate ###");
                ontologia.getClassesInSignature().stream().forEach(owlClass -> log(" - " + owlClass.getIRI().getFragment()));
                String baseIRI = ontologia.getOntologyID().getOntologyIRI()
                    .map(iri -> iri.toString())
                    .orElse("http://www.semanticweb.org/utente/ontologies/2025/11/untitled-ontology-13");

                // Assicurati che l'IRI finisca con # o /
                if (!baseIRI.endsWith("#") && !baseIRI.endsWith("/")) {
                    baseIRI += "#";
                }

                // 4. Inizializzazione del Reasoner (Openllet/Pellet per SWRL)
                // Usiamo la Factory perché il generatore di spiegazioni deve poterne creare nuove istanze
                OWLReasonerFactory reasonerFactory = OpenlletReasonerFactory.getInstance();
                OWLReasoner reasoner = reasonerFactory.createReasoner(ontologia, new SimpleConfiguration());

                // Verifica coerenza iniziale
                if (!reasoner.isConsistent()) {
                    log("L'ontologia è inconsistente!");
                    return;
                }

                // 1. Setup Classi Target e Generatore
                DefaultExplanationGenerator gen = new DefaultExplanationGenerator(man, reasonerFactory, ontologia, null);

                OWLClass parentCls = df.getOWLClass(IRI.create(baseIRI + parentClassName));
                Set<OWLClass> subClasses = reasoner.getSubClasses(parentCls, true).getFlattened();
                subClasses.remove(df.getOWLNothing());  // [?]

                if (subClasses.isEmpty()) {
                    log("ATTENZIONE: Nessuna sottoclasse trovata per " + parentClassName);
                } else {
                    subClasses.forEach(c -> log("Target trovato: " + c.getIRI().getShortForm()));
                }

                OWLClass[] targetClasses = subClasses.toArray(new OWLClass[0]);

                StringBuilder jsonResult = new StringBuilder("{");
                boolean firstInd = true;

                // 2. Ciclo SOLO sugli individui che appartengono alla classe padre (o sottoclassi)
                Set<OWLNamedIndividual> individuiRilevanti = reasoner.getInstances(parentCls, false).getFlattened();

                for (OWLNamedIndividual ind : individuiRilevanti) {

                    // 1. Gestione virgola tra gli oggetti principali
                    if (!firstInd) jsonResult.append(", ");
                    firstInd = false;

                    // --- Costruzione chiave complessa con attributi ---
                    StringBuilder props = new StringBuilder();
                    props.append("{");  // Apre l'oggetto chiave

                    boolean firstProp = true;  // Serve per non mettere la virgola al primo attributo

                    // Recupera tutte le proprietà dati
                    for (OWLDataPropertyAssertionAxiom dp : ontologia.getDataPropertyAssertionAxioms(ind)) {
                        if (!firstProp) {
                            props.append(", ");
                        }
                        firstProp = false;

                        String pKey = dp.getProperty().asOWLDataProperty().getIRI().getShortForm();
                        String pVal = dp.getObject().getLiteral().replace("\"", "'");

                        props.append("\"").append(pKey).append("\": \"").append(pVal).append("\"");
                    }
                    props.append("}");  // Chiude l'oggetto chiave

                    // Escapiamo le virgolette per la chiave
                    String chiaveComplessa = props.toString().replace("\"", "\\\"");

                    jsonResult.append("\"").append(chiaveComplessa).append("\": {");

                    boolean firstClass = true;

                    // 3. Controllo per entrambe le classi
                    for (OWLClass clsTarget : targetClasses) {
                        String nomeClasse = clsTarget.getIRI().getShortForm();

                        OWLClassAssertionAxiom ax = df.getOWLClassAssertionAxiom(clsTarget, ind);
                        String testoSpiegazione = "Il reasoner NON deduce assiomi inerenti";

                        // 4. Il Reasoner controlla se è vero
                        if (reasoner.isEntailed(ax)) {
                            try {
                                Set<OWLAxiom> explanation = gen.getExplanation(ax);
                                testoSpiegazione = explanation.toString()
                                    .replace("[", "").replace("]", "")
                                    .replace(",", "")
                                    .replaceAll("https?://[^>]*[#/]", "")
                                    .replaceAll("\\b(rdfs|swrla):", "")
                                    .replaceAll("[<>]", "")
                                    .replaceAll("\\^\\^xsd:\\w+", "")
                                    .replaceAll("\"(.*?)\"", "$1")
                                    .replaceAll("Annotation\\(isRuleEnabled.*?\\)", "")
                                    .replaceAll("Annotation\\(comment.*?\\)", "")
                                    .replaceAll("\\s+", " ")
                                    .replace("( ", "(")
                                    .trim();
                            } catch (Exception e) {
                                testoSpiegazione = "Errore getExplanation()";
                            }
                        }

                        if (!firstClass) {
                            jsonResult.append(", ");
                        }
                        firstClass = false;

                        // Puliamo virgolette e backslash per non rompere il JSON
                        String safeSpiegazione = testoSpiegazione.replace("\"", "'").replace("\\", "\\\\");

                        // Formato diretto "Chiave": "Valore" (senza graffe extra attorno)
                        jsonResult.append(String.format("\"%s\": \"%s\"", nomeClasse, safeSpiegazione));
                    }

                    jsonResult.append("}");
                }

                jsonResult.append("}");

                System.out.println(jsonResult.toString());

            } catch (OWLOntologyCreationException e) {  // Caricamento fallisce
                log("ERRORE: Non è stato possibile caricare l'ontologia.");
                e.printStackTrace();
            }
        } else {  // .exists() == false
            log("ERRORE: Il file non è stato trovato.");
            log("Controllare il percorso assoluto: " + fileOntologia.getAbsolutePath());
        }
    }
}
