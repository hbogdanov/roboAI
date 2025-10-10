from pathlib import Path
from datasets import load_dataset
from transformers import (
    T5ForConditionalGeneration, T5TokenizerFast,
    DataCollatorForSeq2Seq, Trainer, TrainingArguments
)

ROOT = Path(__file__).resolve().parents[3]  
DATA_TRAIN = ROOT / "data/train.jsonl"
DATA_VAL   = ROOT / "data/val.jsonl"
OUT_DIR    = Path(__file__).resolve().parent / "t5-plan"

MODEL_NAME = "t5-small"

def main():
    tok = T5TokenizerFast.from_pretrained(MODEL_NAME)
    model = T5ForConditionalGeneration.from_pretrained(MODEL_NAME)

    ds = load_dataset("json", data_files={"train": str(DATA_TRAIN), "val": str(DATA_VAL)})

    def tok_fn(batch):
        x = tok(batch["input_text"], max_length=512, truncation=True)
        y = tok(batch["target_text"], max_length=384, truncation=True)
        x["labels"] = y["input_ids"]
        return x

    ds_tok = ds.map(tok_fn, batched=True, remove_columns=ds["train"].column_names)
    dc = DataCollatorForSeq2Seq(tok, model=model)

    args = TrainingArguments(
        output_dir=str(OUT_DIR),
        per_device_train_batch_size=8,
        per_device_eval_batch_size=8,
        learning_rate=3e-4,
        num_train_epochs=3,
        weight_decay=0.01,
        save_total_limit=2,
        evaluation_strategy="epoch",
        logging_steps=50,
        predict_with_generate=False,
        fp16=False,
    )

    trainer = Trainer(
        model=model, args=args,
        train_dataset=ds_tok["train"], eval_dataset=ds_tok["val"],
        data_collator=dc, tokenizer=tok
    )
    trainer.train()
    trainer.save_model(str(OUT_DIR))
    tok.save_pretrained(str(OUT_DIR))

if __name__ == "__main__":
    main()
