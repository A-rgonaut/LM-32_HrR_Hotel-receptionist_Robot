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
        if (args.length > 4) {
            DEBUG = Boolean.parseBoolean(args[4]);
        }
        if (DEBUG) {
            System.setProperty("org.slf4j.simpleLogger.defaultLogLevel", "info");
        } else {
            System.setProperty("org.slf4j.simpleLogger.defaultLogLevel", "off");
        }
        if (args.length < 4) {
            log("Uso: java -jar reasoner.jar <OwlFile> <IndividualIRI> <PropertyIRI> <Value(true/false)> [<DEBUG(true/false)>]");
            System.exit(1);
        }
        String owlFilePath     = args[0];
        String individualInput = args[1];
        String propertyInput   = args[2];
        String valueInput      = args[3];

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

                // 5. Definizione dell'Assioma da Spiegare
                IRI iriIndividuo;
                if (individualInput.startsWith("http")) {
                    iriIndividuo = IRI.create(individualInput);
                } else {
                    iriIndividuo = IRI.create(baseIRI + individualInput);
                }

                IRI iriProprieta;
                if (propertyInput.startsWith("http")) {
                    iriProprieta = IRI.create(propertyInput);
                } else {
                    iriProprieta = IRI.create(baseIRI + propertyInput);
                }

                OWLNamedIndividual individuo = df.getOWLNamedIndividual(iriIndividuo);
                OWLDataProperty proprieta = df.getOWLDataProperty(iriProprieta);
                boolean valoreAtteso = Boolean.parseBoolean(valueInput);
                OWLLiteral literalValore = df.getOWLLiteral(valoreAtteso);

                OWLDataPropertyAssertionAxiom assiomaDaSpiegare = df.getOWLDataPropertyAssertionAxiom(proprieta, individuo, literalValore);

                log("\n--- Verifica Inferenza ---");
                // Controlliamo prima se il reasoner lo deduce davvero
                if (reasoner.isEntailed(assiomaDaSpiegare)) {

                    log("Generazione della spiegazione in corso (potrebbe richiedere tempo)...\n");

                    // 6. Generazione della Spiegazione
                    // DefaultExplanationGenerator usa una tecnica "black box" creando istanze del reasoner
                    DefaultExplanationGenerator gen = new DefaultExplanationGenerator(
                        man, reasonerFactory, ontologia, null);

                    // Otteniamo il set di assiomi che causano l'inferenza
                    Set<OWLAxiom> spiegazione = gen.getExplanation(assiomaDaSpiegare);
                    String nomeIndividuo = iriIndividuo.getShortForm();
                    String nomeProprieta = iriProprieta.getShortForm();
                    System.out.println("### SPIEGAZIONE (PERCHE " + nomeIndividuo + " HA " + nomeProprieta + " == " + valueInput + "?) ###");
                    System.out.println("--- START AXIOMS ---");
                    System.out.println(spiegazione.toString()
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
                        .trim());
                    System.out.println("--- END AXIOMS ---");
                } else {
                    System.out.println("Il reasoner NON deduce assiomi inerenti");
                }
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
